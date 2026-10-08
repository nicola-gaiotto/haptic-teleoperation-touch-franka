# Deliverable 2 — Commanding the Touch from ROS 2

**Goal (from the supervisor):** do the reverse of Deliverable 1: publish a message from
ROS 2 (from the terminal or from a node) and have the Touch move accordingly, as a first
step towards the feedback channel of the bilateral teleoperation with the Franka.

**Status:** ✅ done — the stylus moves to joint and Cartesian targets published with
`ros2 topic pub`. Gravity compensation is being tuned.

---

## 1. Background: what the Touch can actuate

- 6 DOF measured (3 arm joints with encoders, 3 stylus gimbal angles with
  potentiometers), **3 DOF actuated** (motors on the arm joints only).
- Max force 3.3 N, continuous force 0.88 N (datasheet).
- It is an impedance device: it is commanded in force, not in position. "Moving the
  stylus to a point" means applying a spring-damper force towards that point.

## 2. The problem: no force on Linux

On Ubuntu 24.04 the motors never produced any force — neither with the ROS 2 driver,
nor with the official OpenHaptics examples (`HelloHapticDevice`, `CommandMotorDAC`),
nor with `TouchCheckup`. On the same laptop, under Windows (Touch Device Drivers
2023.10 + OpenHaptics 3.5), forces work: the device is fine.

## 3. Diagnosis

The aim was to find where, between the software and the motors, the command is lost.

| Hypothesis | Test | Result |
|---|---|---|
| USB permissions | `ls -l /dev/bus/usb/...` | `crw-rw-rw-`: excluded |
| Kernel HID driver | `lsusb -t` | no kernel driver bound: the 3D Systems library detaches it and talks to the device directly (normal) |
| Commands do not leave the PC | `usbmon` capture during a 1 N command | commands are sent and acknowledged |
| **Commands carry zero current** | decoding the captured packets | **all 4263 motor packets carry "zero current" (2048) while 1 N is commanded** |
| Only the force → motor conversion is wrong | `CommandMotorDAC` with raw value 3000 | still 2048 in all 35 538 packets: everything is zeroed |
| Motor over-temperature | `~/.3dsystems/config/Device Temperature` | ~27 (room temperature): excluded |
| Missing device setup | `Touch_HeadlessSetup` | no change (not required for HID devices) |
| Driver version | 2024.09.19 vs 2025.12.10 | same behaviour |
| Locale (vendor note) | `LC_ALL=en_US.UTF-8` | same behaviour |
| Config folder (vendor note) | empty `~/.3dsystems/config` | same behaviour |
| Wrong installation | clean reinstall from the official files | same behaviour; Deliverable 1 still works |

**Conclusion:** on this system the 3D Systems Linux library reads the device correctly
but always sends zero current to the motors. The library is closed source, so it cannot
be fixed. Evidence (USB captures) is kept in [`data/usb_captures`](../data/README.md).

## 4. Solution: commanding the motors directly over USB

From the captures, the USB protocol was decoded ([usb_protocol.md](usb_protocol.md)).
A first test sending the motor command directly (bypassing OpenHaptics) showed that
**all three motors respond**. The driver below is built on this.

### Motor characterisation

`tools/usb/characterize_motors.py`, hands-off, command ±400:

| Motor | Encoder | Sign | Movement |
|---|---|---|---|
| 0 | enc0 | opposite | base rotation |
| 1 | enc1 | opposite | stylus up/down |
| 2 | enc2 | same | stylus towards/away from the operator |

### Driver architecture (`ros2/touch_raw_driver`)

| Layer | Content |
|---|---|
| 1. USB | sends report 10 (motor commands), reads report 11 (encoders), ~250 Hz in a dedicated thread |
| 2. Joint control | PID per joint with the measured signs, speed-limited reference, saturation, integral anti-windup |
| 3. Kinematics | encoders → joint angles → position in metres, and analytic inverse kinematics ([calibration.md](calibration.md)) |
| 4. ROS 2 interface | `/touch/pose`, `/touch/target`, `/touch/enable`, `/touch/mode`, ... (see README) |

Safety features: [safety.md](safety.md).

### Joint space first, then Cartesian

1. **Joint space:** the target is given in encoder counts on `/touch/joint_target`.
   With the motors off, a position is recorded by hand; after enabling, publishing
   those counts brings the stylus back there.
2. **Cartesian:** the kinematic model was identified against OpenHaptics (used
   read-only). The node now publishes `/touch/pose` in the same frame as
   `/phantom/pose` and accepts Cartesian targets in metres on `/touch/target`.
   Verified: inkwell position (0.000, 0.088, −0.066) m as in Deliverable 1; a target
   at x = +0.05 m moves the stylus 5 cm to the right.

## 5. Gravity compensation (in progress)

Without compensation the arm falls when released and the controller relies only on
the integral term. A physical model (sines/cosines of the joint angles) was identified
by holding the arm in several poses, approaching each pose from both sides to cancel
static friction ([calibration.md](calibration.md)).

Findings:
- holding the arm near the centre needs motor commands of ~450–500, and some extended
  poses need more than 600 — close to what the motors can sustain continuously;
- therefore the compensation is **partial** (`gravity_gain` ≈ 0.7) and a thermal
  protection caps a motor whose RMS command stays too high
  ([safety.md](safety.md));
- a `float` mode applies only the gravity compensation, so the arm can be moved by
  hand with reduced weight.

## 6. Demo commands

```bash
# Terminal 1
ros2 launch touch_raw_driver touch_raw_driver.launch.py
# Terminal 2
ros2 topic echo /touch/pose --field pose.position
# Terminal 3
ros2 topic pub --once -w 1 /touch/enable std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once -w 1 /touch/target geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 0.0}"
ros2 topic pub --once -w 1 /touch/target geometry_msgs/msg/Point "{x: 0.05, y: 0.0, z: 0.0}"
ros2 topic pub --once -w 1 /touch/mode std_msgs/msg/String "{data: float}"
ros2 topic pub --once -w 1 /touch/enable std_msgs/msg/Bool "{data: false}"   # hold the stylus
```

## 7. Limitations

- The protocol is decoded, not documented: the meaning of some bytes (e.g. the flag byte
  `0x03`) is inferred.
- No temperature model from the vendor: the thermal protection is a conservative
  RMS limit chosen by me.
- Python control loop at ~250 Hz (the vendor loop runs at 1 kHz): fine for position
  control; a C++ loop may be needed for stiff force rendering.
- The stylus orientation is not computed yet.
