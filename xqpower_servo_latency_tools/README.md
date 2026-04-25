# XQPOWER External USB-CAN Tools

This folder contains standalone host-side tools to validate servo command/response and measure latency precisely.

## Files
- `servo_connectivity_probe.py`: safe first motion + ACK check.
- `servo_connectivity_probe_hid.py`: same probe over HID (`/dev/hidrawX`), matching CAN_LIN Android path.
- `servo_connectivity_probe_libusb.py`: direct USB bulk probe (closest to Android USB transfer behavior).
- `canlin_passive_sniffer.py`: passive CAN frame sniff while HITL is running.
- `servo_latency_benchmark.py`: repeated command/feedback benchmark with CSV output.
- `requirements.txt`: Python dependencies.

## Install
```bash
cd m13/xqpower_servo_latency_tools
python3 -m pip install --user -r requirements.txt
```

## 1) Connectivity probe (first movement)
```bash
python3 servo_connectivity_probe.py --port /dev/ttyACM0 --node 1 --move-deg 2.0 --term120
```

## 1b) HID probe (recommended for CAN_LIN_Tool)

CAN_LIN_Tool exposes a HID interface used by Android driver path.
On Linux this is typically `/dev/hidraw4` for VID:PID `2e3c:5750`.

```bash
python3 servo_connectivity_probe_hid.py --hidraw /dev/hidraw4 --node 1 --move-deg 2.0 --term120
```

If permission is denied on hidraw:

```bash
echo 'KERNEL=="hidraw*", ATTRS{idVendor}=="2e3c", ATTRS{idProduct}=="5750", MODE="0660", GROUP="dialout"' | sudo tee /etc/udev/rules.d/99-canlin-hid.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Expected:
- Servo makes a small move and returns to zero.
- ACK latencies are printed.

## 2) Latency benchmark
```bash
python3 servo_latency_benchmark.py \
  --port /dev/ttyACM0 \
  --node 1 \
  --step-deg 3.0 \
  --cycles 8 \
  --term120 \
  --out-csv results/node1_latency.csv
```

Metrics:
- ACK latency
- First feedback latency
- Settling time
- Final tracking error
- p50/p95/p99 summary

## Notes
- Current host must have permission to access `/dev/ttyACM0`.
- Tool follows the CAN_LIN framing currently used by the project driver.

## Optional direct USB bulk probe

```bash
python3 servo_connectivity_probe_libusb.py --node 1 --move-deg 2.0
```

If libusb shows access denied, add a usbfs rule for VID:PID 2e3c:5750 so user can claim interfaces.

## Passive sniff during HITL (key diagnostic)

Run this while HITL is actively driving the servo. It does not send motion commands; it only listens.

```bash
python3 canlin_passive_sniffer.py --kbps 500 --seconds 15
```

Interpretation:
- If frames appear: adapter can see the live bus and we can align standalone command sequence to HITL traffic.
- If zero frames: standalone path issue is physical/mode/bitrate/topology, not command syntax.
