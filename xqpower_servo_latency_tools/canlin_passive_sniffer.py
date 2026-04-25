#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
import usb.core
import usb.util

VID = 0x2E3C
PID = 0x5750


def parse_stream(buf: bytearray):
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


def run(kbps: int, seconds: float) -> int:
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        print('[ERR] CAN_LIN_Tool not found (VID:PID 2e3c:5750)')
        return 1

    try:
        dev.set_configuration()
    except Exception:
        pass

    for iface in (0, 1, 2):
        try:
            if dev.is_kernel_driver_active(iface):
                dev.detach_kernel_driver(iface)
        except Exception:
            pass
        try:
            usb.util.claim_interface(dev, iface)
        except Exception:
            pass

    ep_out = 0x01
    ep_in = 0x81

    lo = kbps & 0xFF
    hi = (kbps >> 8) & 0xFF

    init_cmds = [
        bytes([0x03, 0x01, lo, hi, 0x00, 0x00, 0x12, 0x00, 0x00, 0x05, 0x00]),
        bytes([0x03, 0x05]),
        bytes([0x06, 0x01]),
        bytes([0x03, 0x02, 0x03]),
    ]

    for c in init_cmds:
        dev.write(ep_out, c, timeout=150)
        time.sleep(0.06)

    print(f'[INFO] Passive sniff started on {kbps} kbps for {seconds:.1f}s')
    rx = bytearray()
    count = 0
    start = time.time()

    while time.time() - start < seconds:
        try:
            d = bytes(dev.read(ep_in, 256, timeout=120))
        except usb.core.USBTimeoutError:
            continue
        except Exception as e:
            print('[ERR] read failed:', e)
            break

        if not d:
            continue

        rx.extend(d)
        for can_id, dlc, payload in parse_stream(rx):
            count += 1
            print(f'[{count:05d}] id=0x{can_id:03X} dlc={dlc} data={payload.hex()}')

    print(f'[DONE] Total frames: {count}')

    for iface in (0, 1, 2):
        try:
            usb.util.release_interface(dev, iface)
        except Exception:
            pass

    return 0


def main() -> int:
    p = argparse.ArgumentParser(description='CAN_LIN passive frame sniffer')
    p.add_argument('--kbps', type=int, default=500)
    p.add_argument('--seconds', type=float, default=10.0)
    args = p.parse_args()
    return run(args.kbps, args.seconds)


if __name__ == '__main__':
    raise SystemExit(main())
