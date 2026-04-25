#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
from typing import Optional

import usb.core
import usb.util

VID = 0x2E3C
PID = 0x5750
XQPOWER_SDO_TX_BASE = 0x600
XQPOWER_SDO_RX_BASE = 0x580


def can_frame(can_id: int, data: bytes) -> bytes:
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
    out = []
    rd = 0
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


def sdo_set_pos(deg: float) -> bytes:
    raw = int(round(deg * 18.0))
    raw = max(-32768, min(32767, raw))
    lo = raw & 0xFF
    hi = (raw >> 8) & 0xFF
    return bytes([0x22, 0x03, 0x60, 0x00, lo, hi, 0x00, 0x00])


def wait_ack(dev, ep_in: int, node: int, sent_ns: int, timeout_s: float, rxbuf: bytearray) -> Optional[int]:
    target = XQPOWER_SDO_RX_BASE + node
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            d = bytes(dev.read(ep_in, 256, timeout=60))
        except usb.core.USBTimeoutError:
            continue
        if not d:
            continue
        rxbuf.extend(d)
        for can_id, dlc, data in parse_stream(rxbuf):
            if can_id == target and dlc >= 4 and data[0] == 0x60:
                return time.perf_counter_ns() - sent_ns
    return None


def run(node: int, move_deg: float):
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        print("[ERR] device not found")
        return 1

    try:
        dev.set_configuration()
    except Exception:
        pass

    # Mirror Android low-level path: detach/claim interfaces explicitly.
    claimed = []
    for iface in (0, 1, 2):
        try:
            if dev.is_kernel_driver_active(iface):
                dev.detach_kernel_driver(iface)
        except Exception:
            pass
        try:
            usb.util.claim_interface(dev, iface)
            claimed.append(iface)
        except Exception:
            pass

    # Try CDC data first, then HID interrupt pair.
    endpoint_candidates = [
        (1, 0x01, 0x81),
        (2, 0x03, 0x83),
    ]

    chosen = None
    for _, ep_out, ep_in in endpoint_candidates:
        try:
            dev.write(ep_out, bytes([0x03, 0x02, 0x03]), timeout=150)
            chosen = (ep_out, ep_in)
            break
        except Exception:
            continue

    if chosen is None:
        for iface in claimed:
            try:
                usb.util.release_interface(dev, iface)
            except Exception:
                pass
        print("[ERR] could not write any endpoint")
        return 2

    ep_out, ep_in = chosen
    print(f"[INFO] endpoints out=0x{ep_out:02X} in=0x{ep_in:02X}")

    for c in (
        bytes([0x03, 0x01, 0xF4, 0x01, 0x00, 0x00, 0x12, 0x00, 0x00, 0x05, 0x00]),
        bytes([0x03, 0x05]),
        bytes([0x06, 0x01]),
        bytes([0x03, 0x02, 0x03]),
    ):
        dev.write(ep_out, c, timeout=150)
        time.sleep(0.08)

    warm = bytes([0xAA, 0xC8, 0x25, 0x06, 0x22, 0x03, 0x60, 0x00, 0xE8, 0x03, 0x00, 0x00, 0x55])
    for _ in range(5):
        dev.write(ep_out, warm, timeout=150)
        time.sleep(0.2)

    dev.write(ep_out, can_frame(0x000, bytes([0x01, node])), timeout=150)
    time.sleep(0.05)

    rxbuf = bytearray()
    cmd_id = XQPOWER_SDO_TX_BASE + node

    t1 = time.perf_counter_ns()
    dev.write(ep_out, can_frame(cmd_id, sdo_set_pos(move_deg)), timeout=150)
    a1 = wait_ack(dev, ep_in, node, t1, 0.8, rxbuf)

    time.sleep(0.35)

    t2 = time.perf_counter_ns()
    dev.write(ep_out, can_frame(cmd_id, sdo_set_pos(0.0)), timeout=150)
    a2 = wait_ack(dev, ep_in, node, t2, 0.8, rxbuf)

    if a1 is not None:
        print(f"[OK] ACK1 {a1/1e6:.3f} ms")
    else:
        print("[WARN] ACK1 none")

    if a2 is not None:
        print(f"[OK] ACK2 {a2/1e6:.3f} ms")
    else:
        print("[WARN] ACK2 none")

    if a1 is None and a2 is None:
        for iface in claimed:
            try:
                usb.util.release_interface(dev, iface)
            except Exception:
                pass
        print("[FAIL] no ACK")
        return 3

    for iface in claimed:
        try:
            usb.util.release_interface(dev, iface)
        except Exception:
            pass

    print("[PASS] connectivity ok")
    return 0


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--node", type=int, default=1)
    p.add_argument("--move-deg", type=float, default=2.0)
    a = p.parse_args()
    return run(a.node, a.move_deg)


if __name__ == "__main__":
    raise SystemExit(main())
