#!/usr/bin/env python3
"""
XQPOWER CAN servo latency and tracking benchmark via CAN_LIN_Tool.

Measures per command:
- ACK latency (SDO write ACK)
- First feedback latency (first observed position change)
- Settling time (within tolerance and remains stable for hold window)
- Final position error

Outputs CSV and summary stats.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import serial


XQPOWER_SDO_TX_BASE = 0x600
XQPOWER_SDO_RX_BASE = 0x580
XQPOWER_PDO_RX_BASE = 0x180


@dataclass
class CommandResult:
    seq: int
    node: int
    cmd_deg: float
    cmd_ts_ns: int
    ack_ts_ns: Optional[int]
    first_fb_ts_ns: Optional[int]
    settle_ts_ns: Optional[int]
    final_fb_deg: Optional[float]
    settle_error_deg: Optional[float]


class CanLinAdapter:
    def __init__(self, port: str, baud: int, timeout_s: float = 0.05) -> None:
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout_s, write_timeout=0.2)
        self._rx = bytearray()

    def close(self) -> None:
        self.ser.close()

    def write_raw(self, data: bytes) -> None:
        self.ser.write(data)
        self.ser.flush()

    def configure(self, term120: bool) -> None:
        cmds = [
            bytes([0x03, 0x01, 0xF4, 0x01, 0x00, 0x00, 0x12, 0x00, 0x00, 0x05, 0x00]),  # 500k
            bytes([0x03, 0x05]),  # save
            bytes([0x06, 0x01 if term120 else 0x00]),  # termination
            bytes([0x03, 0x02, 0x03]),  # receive all
        ]
        for c in cmds:
            self.write_raw(c)
            time.sleep(0.08)

    def send_can(self, can_id: int, payload: bytes) -> int:
        if len(payload) > 8:
            raise ValueError("payload > 8")
        f = bytearray(20)
        f[0] = 0x01
        f[1] = can_id & 0xFF
        f[2] = (can_id >> 8) & 0xFF
        f[9] = 0x00
        f[10] = 0x00
        f[11] = len(payload)
        f[12:12 + len(payload)] = payload
        ts = time.perf_counter_ns()
        self.write_raw(bytes(f))
        return ts

    def poll_events(self) -> List[tuple[int, int, bytes, int]]:
        """Return list of (ts_ns, can_id, data, dlc)."""
        out: List[tuple[int, int, bytes, int]] = []
        chunk = self.ser.read(512)
        if chunk:
            self._rx.extend(chunk)

        rd = 0
        while rd + 17 <= len(self._rx):
            dlc = self._rx[rd]
            if dlc > 8:
                rd += 1
                continue
            can_id = self._rx[rd + 2] | (self._rx[rd + 3] << 8)
            data = bytes(self._rx[rd + 9:rd + 9 + dlc])
            out.append((time.perf_counter_ns(), can_id, data, dlc))
            rd += 17

        if rd > 0:
            del self._rx[:rd]

        return out


def build_sdo_set_position(angle_deg: float) -> bytes:
    raw = int(round(angle_deg * 18.0))
    if raw < -32768 or raw > 32767:
        raise ValueError("position raw out of range")
    return struct.pack("<BBBBhBB", 0x22, 0x03, 0x60, 0x00, raw, 0x00, 0x00)


def build_sdo_read_position() -> bytes:
    return bytes([0x40, 0x02, 0x60, 0x00, 0, 0, 0, 0])


def build_nmt_start(node_id: int) -> bytes:
    return bytes([0x01, node_id])


def percentile(values: List[float], p: float) -> float:
    if not values:
        return float("nan")
    x = sorted(values)
    k = (len(x) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return x[int(k)]
    return x[f] * (c - k) + x[c] * (k - f)


def run_benchmark(args: argparse.Namespace) -> int:
    adapter = CanLinAdapter(args.port, args.baud)
    results: List[CommandResult] = []

    last_fb_deg: Dict[int, float] = {}

    try:
        print(f"[INFO] Open {args.port}, node={args.node}, cycles={args.cycles}")
        adapter.configure(term120=args.term120)

        adapter.send_can(0x000, build_nmt_start(args.node))
        time.sleep(0.05)

        sequence = [0.0, args.step_deg, 0.0, -args.step_deg, 0.0]
        seq_no = 0

        for c in range(args.cycles):
            for target in sequence:
                seq_no += 1
                cmd_id = XQPOWER_SDO_TX_BASE + args.node
                cmd = build_sdo_set_position(target)
                cmd_ts = adapter.send_can(cmd_id, cmd)

                ack_ts: Optional[int] = None
                first_fb_ts: Optional[int] = None
                settle_ts: Optional[int] = None
                final_fb: Optional[float] = None
                settle_err: Optional[float] = None
                stable_since: Optional[int] = None

                deadline = time.monotonic() + args.window_s

                while time.monotonic() < deadline:
                    # Request fresh position periodically to guarantee feedback flow.
                    adapter.send_can(cmd_id, build_sdo_read_position())
                    for ts_ns, can_id, data, dlc in adapter.poll_events():
                        if can_id == XQPOWER_SDO_RX_BASE + args.node and dlc >= 4 and data[0] == 0x60 and ack_ts is None:
                            ack_ts = ts_ns

                        pos_deg: Optional[float] = None
                        if can_id == XQPOWER_SDO_RX_BASE + args.node and dlc >= 6 and data[0] in (0x42, 0x43, 0x4B, 0x4F) and data[1] == 0x02 and data[2] == 0x60:
                            raw = int.from_bytes(data[4:6], "little", signed=True)
                            pos_deg = raw / 18.0
                        elif can_id == XQPOWER_PDO_RX_BASE + args.node and dlc >= 2:
                            raw = int.from_bytes(data[0:2], "little", signed=True)
                            pos_deg = raw / 18.0

                        if pos_deg is not None:
                            prev = last_fb_deg.get(args.node)
                            last_fb_deg[args.node] = pos_deg
                            final_fb = pos_deg

                            if first_fb_ts is None and prev is not None and abs(pos_deg - prev) >= args.fb_change_threshold_deg:
                                first_fb_ts = ts_ns

                            err = target - pos_deg
                            if abs(err) <= args.settle_tol_deg:
                                if stable_since is None:
                                    stable_since = ts_ns
                                elif (ts_ns - stable_since) >= int(args.settle_hold_s * 1e9) and settle_ts is None:
                                    settle_ts = ts_ns
                                    settle_err = err
                            else:
                                stable_since = None

                    time.sleep(args.poll_interval_s)

                results.append(
                    CommandResult(
                        seq=seq_no,
                        node=args.node,
                        cmd_deg=target,
                        cmd_ts_ns=cmd_ts,
                        ack_ts_ns=ack_ts,
                        first_fb_ts_ns=first_fb_ts,
                        settle_ts_ns=settle_ts,
                        final_fb_deg=final_fb,
                        settle_error_deg=settle_err,
                    )
                )

                print(f"[STEP {seq_no}] cmd={target:+.2f} deg ack={'ok' if ack_ts else 'none'} fb={'ok' if first_fb_ts else 'none'} settle={'ok' if settle_ts else 'none'}")
                time.sleep(args.step_gap_s)

    finally:
        adapter.close()

    out = Path(args.out_csv)
    out.parent.mkdir(parents=True, exist_ok=True)

    with out.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "seq", "node", "cmd_deg", "cmd_ts_ns", "ack_ts_ns", "ack_latency_ms",
            "first_fb_ts_ns", "first_fb_latency_ms", "settle_ts_ns", "settle_time_ms",
            "final_fb_deg", "final_err_deg", "settle_err_deg"
        ])
        for r in results:
            ack_ms = ((r.ack_ts_ns - r.cmd_ts_ns) / 1e6) if r.ack_ts_ns else None
            fb_ms = ((r.first_fb_ts_ns - r.cmd_ts_ns) / 1e6) if r.first_fb_ts_ns else None
            st_ms = ((r.settle_ts_ns - r.cmd_ts_ns) / 1e6) if r.settle_ts_ns else None
            final_err = (r.cmd_deg - r.final_fb_deg) if r.final_fb_deg is not None else None
            w.writerow([
                r.seq, r.node, f"{r.cmd_deg:.6f}", r.cmd_ts_ns, r.ack_ts_ns,
                f"{ack_ms:.6f}" if ack_ms is not None else "",
                r.first_fb_ts_ns, f"{fb_ms:.6f}" if fb_ms is not None else "",
                r.settle_ts_ns, f"{st_ms:.6f}" if st_ms is not None else "",
                f"{r.final_fb_deg:.6f}" if r.final_fb_deg is not None else "",
                f"{final_err:.6f}" if final_err is not None else "",
                f"{r.settle_error_deg:.6f}" if r.settle_error_deg is not None else "",
            ])

    ack_vals = [((r.ack_ts_ns - r.cmd_ts_ns) / 1e6) for r in results if r.ack_ts_ns]
    fb_vals = [((r.first_fb_ts_ns - r.cmd_ts_ns) / 1e6) for r in results if r.first_fb_ts_ns]
    st_vals = [((r.settle_ts_ns - r.cmd_ts_ns) / 1e6) for r in results if r.settle_ts_ns]

    def summary(name: str, vals: List[float]) -> None:
        if not vals:
            print(f"[SUMMARY] {name}: no data")
            return
        print(
            f"[SUMMARY] {name}: n={len(vals)} mean={statistics.fmean(vals):.3f} ms "
            f"p50={percentile(vals,50):.3f} p95={percentile(vals,95):.3f} p99={percentile(vals,99):.3f} max={max(vals):.3f}"
        )

    print(f"[DONE] CSV: {out}")
    summary("ACK latency", ack_vals)
    summary("First feedback latency", fb_vals)
    summary("Settle time", st_vals)

    return 0


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="XQPOWER latency benchmark")
    p.add_argument("--port", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=2_000_000)
    p.add_argument("--node", type=int, default=1)
    p.add_argument("--step-deg", type=float, default=3.0)
    p.add_argument("--cycles", type=int, default=8)
    p.add_argument("--window-s", type=float, default=0.9)
    p.add_argument("--step-gap-s", type=float, default=0.15)
    p.add_argument("--poll-interval-s", type=float, default=0.02)
    p.add_argument("--fb-change-threshold-deg", type=float, default=0.15)
    p.add_argument("--settle-tol-deg", type=float, default=0.30)
    p.add_argument("--settle-hold-s", type=float, default=0.08)
    p.add_argument("--term120", action="store_true")
    p.add_argument("--out-csv", default="results/latency_benchmark.csv")
    return p


def main() -> int:
    args = parser().parse_args()
    if args.node < 1 or args.node > 127:
        print("[ERR] node must be 1..127")
        return 1
    if args.step_deg <= 0 or args.step_deg > 15:
        print("[ERR] step-deg must be in (0,15]")
        return 1
    return run_benchmark(args)


if __name__ == "__main__":
    sys.exit(main())
