#!/usr/bin/env python3
"""
Safe connectivity probe for XQPOWER CAN servos via CAN_LIN_Tool on Linux.

Phase 1 goal: verify command -> servo response path with minimal motion.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import List, Optional

import serial


XQPOWER_SDO_TX_BASE = 0x600
XQPOWER_SDO_RX_BASE = 0x580


@dataclass
class CanEvent:
    ts_ns: int
    can_id: int
    dlc: int
    data: bytes


def build_canlin_frame(can_id: int, data: bytes) -> bytes:
    """Build 20-byte CAN_LIN frame used by existing driver for standard CAN IDs."""
    if not (0 <= can_id <= 0x7FF):
        raise ValueError("can_id must be 11-bit")
    if len(data) > 8:
        raise ValueError("data length must be <= 8")

    frame = bytearray(20)
    frame[0] = 0x01  # command: CAN frame
    frame[1] = can_id & 0xFF
    frame[2] = (can_id >> 8) & 0xFF
    frame[3] = 0x00
    frame[4] = 0x00
    frame[9] = 0x00   # standard ID
    frame[10] = 0x00  # data frame
    frame[11] = len(data)
    frame[12:12 + len(data)] = data
    return bytes(frame)


def parse_canlin_stream(buf: bytearray) -> List[CanEvent]:
    """
    Parse rx stream according to observed layout in XqpowerCan.cpp:
    [0]=dlc, [1]=flags, [2:4]=id_le, [4:8]=ext, [8]=dlc2, [9:17]=data
    total size = 17 bytes.
    """
    out: List[CanEvent] = []
    rd = 0
    now_ns = time.perf_counter_ns()

    while rd + 17 <= len(buf):
        dlc = buf[rd]
        if dlc > 8:
            rd += 1
            continue

        can_id = buf[rd + 2] | (buf[rd + 3] << 8)
        data_start = rd + 9
        data = bytes(buf[data_start:data_start + dlc])
        out.append(CanEvent(ts_ns=now_ns, can_id=can_id, dlc=dlc, data=data))
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


def build_sdo_write_set_position(angle_deg: float) -> bytes:
    """Write 0x6003 with raw position (18 units per degree)."""
    raw = int(round(angle_deg * 18.0))
    if raw < -32768 or raw > 32767:
        raise ValueError("angle out of int16 raw range")

    return struct.pack("<BBBBhBB", 0x22, 0x03, 0x60, 0x00, raw, 0x00, 0x00)


def build_nmt_start(node_id: int) -> tuple[int, bytes]:
    return 0x000, bytes([0x01, node_id])


def wait_for_ack(
    ser: serial.Serial,
    node_id: int,
    sent_ns: int,
    timeout_s: float,
    rx_buf: bytearray,
) -> Optional[int]:
    target_id = XQPOWER_SDO_RX_BASE + node_id
    end = time.monotonic() + timeout_s

    while time.monotonic() < end:
        chunk = ser.read(256)
        if chunk:
            rx_buf.extend(chunk)
            events = parse_canlin_stream(rx_buf)
            for ev in events:
                if ev.can_id == target_id and ev.dlc >= 4 and ev.data[0] == 0x60:
                    return ev.ts_ns - sent_ns

    return None


def send_can(ser: serial.Serial, can_id: int, payload: bytes) -> int:
    frame = build_canlin_frame(can_id, payload)
    ts_ns = time.perf_counter_ns()
    wr = ser.write(frame)
    ser.flush()
    if wr != len(frame):
        raise RuntimeError(f"short write {wr}/{len(frame)}")
    return ts_ns


def run_probe(args: argparse.Namespace) -> int:
    print(f"[INFO] Opening {args.port} baud={args.baud}")
    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.05, write_timeout=0.2)
    rx_buf = bytearray()

    try:
        term_state = "ON" if args.term120 else "OFF"
        print(f"[INFO] Configuring CAN_LIN tool for 500k + receive-all + 120R={term_state}")
        for cmd in (cmd_set_500k(), cmd_save(), cmd_term_120r(args.term120), cmd_receive_all()):
            ser.write(cmd)
            ser.flush()
            time.sleep(0.08)

        print(f"[INFO] NMT start node={args.node}")
        nmt_id, nmt_payload = build_nmt_start(args.node)
        send_can(ser, nmt_id, nmt_payload)
        time.sleep(0.05)

        print(f"[INFO] Move to +{args.move_deg:.2f} deg")
        payload = build_sdo_write_set_position(args.move_deg)
        can_id = XQPOWER_SDO_TX_BASE + args.node
        sent1 = send_can(ser, can_id, payload)
        ack1 = wait_for_ack(ser, args.node, sent1, args.ack_timeout_s, rx_buf)

        time.sleep(args.hold_s)

        print("[INFO] Move back to 0.00 deg")
        payload_zero = build_sdo_write_set_position(0.0)
        sent2 = send_can(ser, can_id, payload_zero)
        ack2 = wait_for_ack(ser, args.node, sent2, args.ack_timeout_s, rx_buf)

        if ack1 is not None:
            print(f"[OK] ACK1 latency: {ack1 / 1e6:.3f} ms")
        else:
            print("[WARN] No ACK for first move within timeout")

        if ack2 is not None:
            print(f"[OK] ACK2 latency: {ack2 / 1e6:.3f} ms")
        else:
            print("[WARN] No ACK for return-to-zero within timeout")

        if ack1 is None and ack2 is None:
            print("[FAIL] No servo ACK observed. Check node ID, wiring, CAN power, termination.")
            return 2

        print("[PASS] Connectivity path is alive (command -> servo response)")
        return 0

    finally:
        ser.close()


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="XQPOWER servo connectivity probe (safe motion)")
    p.add_argument("--port", default="/dev/ttyACM0", help="CAN_LIN serial device")
    p.add_argument("--baud", type=int, default=2000000, help="Serial baud (CDC may ignore this)")
    p.add_argument("--node", type=int, default=1, help="Servo CAN node ID")
    p.add_argument("--move-deg", type=float, default=2.0, help="Small safe move angle")
    p.add_argument("--hold-s", type=float, default=0.35, help="Hold duration before return-to-zero")
    p.add_argument("--ack-timeout-s", type=float, default=0.6, help="Timeout waiting for SDO ACK")
    p.add_argument("--term120", action="store_true", help="Enable 120R termination on adapter")
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    if args.move_deg < 0.1 or args.move_deg > 10.0:
        print("[ERR] --move-deg out of safe probe range (0.1..10.0)")
        return 1
    if args.node < 1 or args.node > 127:
        print("[ERR] --node must be in [1..127]")
        return 1
    return run_probe(args)


if __name__ == "__main__":
    sys.exit(main())
