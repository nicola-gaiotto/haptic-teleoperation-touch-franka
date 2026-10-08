# Recorded data

| Folder | Content |
|---|---|
| `usb_captures/` | usbmon captures that show the zero-current problem of the vendor library |
| `kinematics_logs/` | `kin_log.csv` used to identify the kinematic model |

Captures of the problem (Deliverable 2):

| File | What it shows |
|---|---|
| `usbmon_K2.txt` | `force_probe2 K y 1.0` (1 N commanded): all 4263 motor packets carry 2048 (zero current) |
| `usbmon_DAC.txt` | `CommandMotorDAC` with raw value 3000: all 35 538 motor packets carry 2048 |

Keep files small (a few MB). `.pcap` files are ignored by git (see `.gitignore`).

Analysis command:

```bash
grep "S Io:3:003:2" usbmon_K2.txt | awk '{print $9, $10}' | sort | uniq -c
```
