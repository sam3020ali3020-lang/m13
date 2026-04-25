#!/usr/bin/env python3
"""
Connectivity probe for XQPOWER via CAN_LIN_Tool HID interface (/dev/hidrawX).

This follows the Android path more closely because CAN_LIN_Tool exposes a HID
interface (EP 0x03/0x83), while /dev/ttyACM0 uses CDC and may not carry CAN data.
"""

from __future__ import annotations

import argparse
import os
import select
import struct
import sys
import time
from typing import Optional

XQPOWER_SDO_TX_BASE = 0x600
XQPOWER_SDO_RX_BASE = 0x580


class HidAdapter:
    def __init__(self, dev: str) -> None:
        self.dev = dev
        self.fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
        # Report descriptor says Input/Output report count is 0x3E (62 bytes).
        self.report_size = 62

    def close(self) -> None:
        os.close(self.fd)

    def _write_raw(self, payload: bytes) -> int:
        return os.write(self.fd, payload)

    def write_packet(self, payload: bytes) -> int:
        packet = payload[:self.report_size].ljust(self.report_size, b"\x00")
        return self._write_raw(packet)

    def read_packet(self, timeout_s: float = 0.05) -> bytes:
        r, _, _ = select.select([self.fd], [], [], timeout_s)
        if not r:
            return b""
        data = os.read(self.fd, self.report_size)
        if not data:
            return b""
        return data


def canlin_frame(can_id: int, data: bytes) -> bytes:
    f = bytearray(20)
    f[0] = 0x01
    f[1] = can_id & 0xFF
    f[2] = (can_id >> 8) & 0xFF
    f[9] = 0x00
    f[10] = 0x00
    f[11] = len(data)
    f[12:12 + len(data)] = data
    return bytes(f)


def parse_stream(buf: bytearray):
    rd = 0
    out = []
    while rd + 17 <= len(buf):
        dlc = buf[rd]
        if dlc > 8:
            rd += 1
            continue
        can_id = buf[rd + 2] | (buf[rd + 3] << 8)
        data = bytes(buf[rd + 9: rd + 9 + dlc])
        out.append((can_id, dlc, data))
        rd += 17
    if rd > 0:
        del buf[:rd]
    return out


def cmd_set_500k() -> bytes:
    return bytes([0x03, 0x01, 0xF4, 0x01, 0x00, 0x00, 0x12, 0x00, 0x00, 0x05, 0x00])


def cmd_save() -> bytes:
    return bytes([0x03, 0x05])


def cmd_term120(on: bool) -> bytes:
    return bytes([0x06, 0x01 if on else 0x00])


def cmd_receive_all() -> bytes:
    return bytes([0x03, 0x02, 0x03])


def sdo_set_pos(angle_deg: float) -> bytes:
    raw = int(round(angle_deg * 18.0))
    if raw < -32768 or raw > 32767:
        raise ValueError("angle out of range")
    return struct.pack("<BBBBhBB", 0x22, 0x03, 0x60, 0x00, raw, 0x00, 0x00)


def run(args: argparse.Namespace) -> int:
    ad = HidAdapter(args.hidraw)
    rxbuf = bytearray()

    try:
        print(f"[INFO] Open HID {args.hidraw}")
        term_state = "ON" if args.term120 else "OFF"
        print(f"[INFO] Configure CAN_LIN: 500k, recv-all, 120R={term_state}")

        for c in (cmd_set_500k(), cmd_save(), cmd_term120(args.term120), cmd_receive_all()):
            ad.write_packet(c)
            time.sleep(0.08)

        # Mirror PX4 driver warm-up sequence. These frames target a dummy node
        # and are used to wake some adapters before normal traffic starts.
        warm = bytes([
            0xAA, 0xC8, 0x25, 0x06,
            0x22, 0x03, 0x60, 0x00,
            0xE8, 0x03, 0x00, 0x00,
            0x55,
        ])
        for _ in range(5):
            ad.write_packet(warm)
            time.sleep(0.2)

        # Flush any pending bytes and show first chunks for diagnostics.
        for _ in range(4):
            d = ad.read_packet(0.03)
            if d:
                print(f"[DBG] pre-rx {len(d)}: {d[:32].hex()}")

        nmt = canlin_frame(0x000, bytes([0x01, args.node]))
        ad.write_packet(nmt)
        time.sleep(0.05)

        cmd_id = XQPOWER_SDO_TX_BASE + args.node

        print(f"[INFO] Move +{args.move_deg:.2f} deg")
        t1 = time.perf_counter_ns()
        ad.write_packet(canlin_frame(cmd_id, sdo_set_pos(args.move_deg)))

        ack1 = wait_ack(ad, rxbuf, args.node, t1, args.ack_timeout_s)

        time.sleep(args.hold_s)

        print("[INFO] Move 0.00 deg")
        t2 = time.perf_counter_ns()
        ad.write_packet(canlin_frame(cmd_id, sdo_set_pos(0.0)))

        ack2 = wait_ack(ad, rxbuf, args.node, t2, args.ack_timeout_s)

        if ack1 is not None:
            print(f"[OK] ACK1 latency: {ack1/1e6:.3f} ms")
        else:
            print("[WARN] No ACK1")

        if ack2 is not None:
            print(f"[OK] ACK2 latency: {ack2/1e6:.3f} ms")
        else:
            print("[WARN] No ACK2")

        if ack1 is None and ack2 is None:
            print("[FAIL] No servo ACK via HID path")
            return 2

        print("[PASS] Connectivity alive via HID path")
        return 0

    finally:
        ad.close()


def wait_ack(ad: HidAdapter, rxbuf: bytearray, node: int, sent_ns: int, timeout_s: float) -> Optional[int]:
    end = time.monotonic() + timeout_s
    target_id = XQPOWER_SDO_RX_BASE + node

    while time.monotonic() < end:
        d = ad.read_packet(0.05)
        if not d:
            continue
        rxbuf.extend(d)
        for can_id, dlc, data in parse_stream(rxbuf):
            if can_id == target_id and dlc >= 4 and data[0] == 0x60:
                return time.perf_counter_ns() - sent_ns
    return None


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="XQPOWER connectivity probe over HID")
    p.add_argument("--hidraw", default="/dev/hidraw4")
    p.add_argument("--node", type=int, default=1)
    p.add_argument("--move-deg", type=float, default=2.0)
    p.add_argument("--hold-s", type=float, default=0.35)
    p.add_argument("--ack-timeout-s", type=float, default=0.8)
    p.add_argument("--term120", action="store_true")
    return p


def main() -> int:
    args = build_parser().parse_args()
    if args.node < 1 or args.node > 127:
        print("[ERR] node must be 1..127")
        return 1
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
