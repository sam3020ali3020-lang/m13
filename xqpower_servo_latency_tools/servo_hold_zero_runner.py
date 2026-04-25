#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import time
from typing import List

import serial

XQPOWER_SDO_TX_BASE = 0x600
XQPOWER_SDO_RX_BASE = 0x580


def build_canlin_frame(can_id: int, data: bytes) -> bytes:
    frame = bytearray(20)
    frame[0] = 0x01
    frame[1] = can_id & 0xFF
    frame[2] = (can_id >> 8) & 0xFF
    frame[9] = 0x00
    frame[10] = 0x00
    frame[11] = len(data)
    frame[12:12 + len(data)] = data
    return bytes(frame)


def parse_canlin_stream(buf: bytearray):
    out = []
    rd = 0
    while rd + 17 <= len(buf):
        dlc = buf[rd]
        if dlc > 8:
            rd += 1
            continue
        can_id = buf[rd + 2] | (buf[rd + 3] << 8)
        data = bytes(buf[rd + 9:rd + 9 + dlc])
        out.append((can_id, dlc, data))
        rd += 17
    if rd > 0:
        del buf[:rd]
    return out


def cmd_set_500k() -> bytes:
    return bytes([0x03, 0x01, 0xF4, 0x01, 0x00, 0x00, 0x12, 0x00, 0x00, 0x05, 0x00])


def cmd_save() -> bytes:
    return bytes([0x03, 0x05])


def cmd_term_120r(enable: bool) -> bytes:
    return bytes([0x06, 0x01 if enable else 0x00])


def cmd_receive_all() -> bytes:
    return bytes([0x03, 0x02, 0x03])


def sdo_set_position(angle_deg: float) -> bytes:
    raw = int(round(angle_deg * 18.0))
    if raw < -32768:
        raw = -32768
    if raw > 32767:
        raw = 32767
    return struct.pack("<BBBBhBB", 0x22, 0x03, 0x60, 0x00, raw, 0x00, 0x00)


def sdo_set_report_interval(ms: int) -> bytes:
    if ms < 10:
        ms = 10
    if ms > 255:
        ms = 255
    return bytes([0x22, 0x00, 0x22, 0x00, ms, 0x00, 0x00, 0x00])


def send_can(ser: serial.Serial, can_id: int, payload: bytes) -> None:
    frame = build_canlin_frame(can_id, payload)
    wr = ser.write(frame)
    ser.flush()
    if wr != len(frame):
        raise RuntimeError(f"short write {wr}/{len(frame)}")


def parse_nodes(text: str) -> List[int]:
    nodes = []
    for p in text.split(","):
        p = p.strip()
        if not p:
            continue
        n = int(p)
        if n < 1 or n > 127:
            raise ValueError("node out of range")
        nodes.append(n)
    if not nodes:
        raise ValueError("empty node list")
    return nodes


def main() -> int:
    ap = argparse.ArgumentParser(description="Keep XQPOWER servos in zero/hold mode (HITL-like)")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=2000000)
    ap.add_argument("--nodes", default="1,2,3,4", help="comma-separated CAN node IDs")
    ap.add_argument("--duration-s", type=float, default=12.0)
    ap.add_argument("--hz", type=float, default=50.0)
    ap.add_argument("--term120", action="store_true")
    ap.add_argument("--interval-ms", type=int, default=20)
    ap.add_argument("--kick-deg", type=float, default=0.0, help="optional one-shot kick angle at startup")
    args = ap.parse_args()

    nodes = parse_nodes(args.nodes)
    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.01, write_timeout=0.2)
    rx_buf = bytearray()

    print(f"[INFO] open {args.port} nodes={nodes} hz={args.hz:.1f}")

    try:
        for cmd in (cmd_set_500k(), cmd_save(), cmd_term_120r(args.term120), cmd_receive_all()):
            ser.write(cmd)
            ser.flush()
            time.sleep(0.08)

        # Optional warm-up frame from Android driver behavior.
        warm = bytes([0xAA, 0xC8, 0x25, 0x06, 0x22, 0x03, 0x60, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x55])
        for _ in range(3):
            ser.write(warm)
            ser.flush()
            time.sleep(0.1)

        for n in nodes:
            send_can(ser, 0x000, bytes([0x01, n]))
            time.sleep(0.03)

        for n in nodes:
            send_can(ser, XQPOWER_SDO_TX_BASE + n, sdo_set_report_interval(args.interval_ms))
            time.sleep(0.02)

        if abs(args.kick_deg) > 0.01:
            for n in nodes:
                send_can(ser, XQPOWER_SDO_TX_BASE + n, sdo_set_position(args.kick_deg))
                time.sleep(0.03)
            time.sleep(0.2)

        period = 1.0 / max(args.hz, 1.0)
        end = time.monotonic() + args.duration_s
        tx_count = 0
        ack_count = 0
        pdo_count = 0
        last_nmt = 0.0

        print("[INFO] zero-hold loop started")
        while time.monotonic() < end:
            now = time.monotonic()
            if now - last_nmt > 1.0:
                for n in nodes:
                    send_can(ser, 0x000, bytes([0x01, n]))
                last_nmt = now

            for n in nodes:
                send_can(ser, XQPOWER_SDO_TX_BASE + n, sdo_set_position(0.0))
                tx_count += 1

            chunk = ser.read(512)
            if chunk:
                rx_buf.extend(chunk)
                for can_id, dlc, data in parse_canlin_stream(rx_buf):
                    if XQPOWER_SDO_RX_BASE + 1 <= can_id <= XQPOWER_SDO_RX_BASE + 127:
                        if dlc >= 1 and data[0] == 0x60:
                            ack_count += 1
                        else:
                            pdo_count += 1

            time.sleep(period)

        print(f"[DONE] tx_zero={tx_count} ack={ack_count} pdo_like={pdo_count}")
        if ack_count == 0 and pdo_count == 0:
            print("[WARN] no RX seen from bus")
            return 2

        return 0

    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
