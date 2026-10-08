# USB protocol of the 3D Systems Touch (HID)

> Decoded by observing the traffic of the official software with `usbmon` on my own
> device (USB ID `2988:0304`, Touch Device Driver 2025.12.10). This is **not official
> documentation**: field meanings marked *(inferred)* come from sizes and behaviour,
> not from the vendor. Other firmware versions may differ.

## Device

| Item | Value |
|---|---|
| USB | 2.0 full speed, self-powered |
| Interface | 1 interface, class HID, vendor-defined usage pages (0xFFxx) |
| EP 0x81 IN | interrupt, 64 bytes, interval 1 ms (device → PC) |
| EP 0x02 OUT | interrupt, 64 bytes, interval 1 ms (PC → device) |
| Linux | no kernel driver bound while in use: the software detaches `usbhid` and accesses the device through usbfs (`/dev/bus/usb/...`) |

## Reports

| ID | Direction | Size (payload) | Use |
|---|---|---|---|
| 8 | PC → device (control SET_REPORT) | 59 B | queries (e.g. "give me parameter N") |
| 9 | device → PC (control GET_REPORT) | 103 B | answers to the queries, as text |
| 10 | PC → device (EP 0x02) | 11 B | **motor command**, sent periodically |
| 11 | device → PC (EP 0x81) | 34 B | **device state** (encoders), sent periodically |

### Report 10 — motor command (12 bytes with the ID)

```
offset  size  field
0       1     0x0a                 report ID
1       4     uint32 LE            command counter (incremented every packet)
5       2     uint16 LE            motor 0  (2048 = 0x0800 = zero current)
7       2     uint16 LE            motor 1
9       2     uint16 LE            motor 2
11      1     0x03                 flags (inferred; always 0x03 in the captures)
```

The official library sends one packet about every 2 ms. The direct driver writes
`2048 + command`, with |command| ≤ 600 (hard limit in the code).

### Report 11 — device state (35 bytes with the ID)

```
offset  size     field
0       1        0x0b              report ID
1       4        uint32 LE         device counter
5       4        uint32 LE         echo of the last command counter received
9       13 × 2   int16 LE          enc0..enc5, then status values
```

- `enc0..enc2`: arm joint encoders; `enc3..enc5`: gimbal sensors.
- With the stylus in the inkwell: `enc = [0, 659, -910, ~2048, ~2655, ~2184]`.
- The echo of the command counter shows that the device received each command.
- A further value equal to 1 was seen in the inkwell *(inferred: inkwell switch or
  status flag)*.

### Relation with the OpenHaptics encoders

`HD_CURRENT_ENCODER_VALUES` (OpenHaptics) and the raw report-11 values differ in sign
for two joints:

| Encoder | Raw USB vs OpenHaptics | How it was found |
|---|---|---|
| enc0 | **opposite** | x-direction test (enc0 is 0 in the inkwell) |
| enc1 | **opposite** | inkwell: +659 vs −659; ranges mirrored |
| enc2 | same | inkwell: −910 vs −909 |

### Motor ↔ encoder signs

| Motor | Encoder | Positive command makes the encoder |
|---|---|---|
| 0 | enc0 | decrease |
| 1 | enc1 | decrease |
| 2 | enc2 | increase |

## Start-up queries (reports 8 / 9)

At start-up the library queries the device; the answers are ASCII text
(decoded, truncated by usbmon to 32 bytes):

| Group | Examples of answers |
|---|---|
| Identity | `Default Device - Device Name`, `24079000373 - Serial Number`, `1 - TOUCH` |
| Geometry | `133350 - Arm Length`, capstans X/Y/Z `134030 / 132996 / 132820`, pulleys, `-110000` and `-35000` workspace offsets |
| Calibration | X/Y/Z encoders `02048`, gimbal encoders `04915`, reset angles |
| Motors | X/Y/Z motor type `1`; yaw/roll/pitch motor type `0` (no gimbal motors) |
| Rendering | `00500 (0x01F4) - Stiffness`, `00003 - Damping`, `00000 - Max Torque` |
| Diagnostics | `No diagnostic running`; some diagnostic commands do not exist |

The device stores its own configuration: for HID devices no configuration file is
needed on disk.

## Tools

- `tools/usb/raw_motor_test.py` — first direct test of one motor.
- `tools/usb/characterize_motors.py` — motor/encoder signs.
- Capturing the traffic:

```bash
sudo modprobe usbmon
lsusb -d 2988:0304                       # note bus and device numbers
sudo timeout 40 cat /sys/kernel/debug/usb/usbmon/3u > capture.txt   # bus 3
grep "S Io:3:<device>:2" capture.txt | awk '{print $9, $10}' | sort | uniq -c
```
