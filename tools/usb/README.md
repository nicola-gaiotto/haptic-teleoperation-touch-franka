# USB tools

Direct tests of the Touch motors over USB, without OpenHaptics. Requirements:
`sudo apt install python3-usb`. Close every other program that uses the Touch first.

| Script | Purpose |
|---|---|
| `raw_motor_test.py [motor] [delta]` | 6 s test of one motor: zero → `2048+delta` → zero. Shows the counter echo and the encoders. |
| `characterize_motors.py [delta]` | ~27 s automatic test of the 3 motors in both directions: which encoder each motor moves and with which sign. |

Both are **hands-off** tests: put the stylus on the table, leave space around the arm,
and do not touch the device while they run. Zero current is always sent at the end and
on Ctrl+C. Protocol details: [docs/usb_protocol.md](../../docs/usb_protocol.md).
