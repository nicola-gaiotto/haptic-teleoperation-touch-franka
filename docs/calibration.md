# Calibration

The driver needs two device-specific files, read at start-up from
`~/.touch_raw_driver/` (paths configurable with the parameters `kinematics_file` and
`gravity_file`). The files of my device are versioned in [`calibration/`](../calibration/).

| File | Produced by | Used for |
|---|---|---|
| `kinematics.json` | `tools/calibration/kin_logger` + `kin_fit.py` | `/touch/pose`, `/touch/joint_states`, Cartesian targets |
| `gravity.json` | `tools/calibration/gravity_calibration.py` | gravity compensation |

## 1. Kinematic model

### Idea

OpenHaptics cannot drive the motors on this system, but it **reads** the device
correctly. It is used read-only as a measuring instrument: while the stylus is moved by
hand, `kin_logger` records at 100 Hz the raw encoders, the joint angles and the position
computed by OpenHaptics. `kin_fit.py` then fits:

1. encoders → joint angles (affine, 3 × 3 matrix + offset);
2. joint angles → position (two candidate structures, linear least squares);
3. an end-to-end check, encoders → position.

### Procedure

```bash
cd tools/calibration
make                         # builds kin_logger (needs OpenHaptics)
./kin_logger 60              # stylus in the inkwell for 5 s, then move it everywhere
python3 kin_fit.py kin_log.csv   # writes ~/.touch_raw_driver/kinematics.json
```

### Result on my device

Exact fit (RMS error 0.000° on the angles, 0.00 mm on the position):

```
q0 = -4.112e-4 · e0      q1 = -4.0803e-4 · e1      q2 = 4.0749e-4 · e2     [rad]
(e = OpenHaptics encoders; raw USB: e0 and e1 have the opposite sign)

r = L1 cos q1 + L2 sin q2
x = -sin(q0) · r + x0
y =  L1 sin q1 - L2 cos q2 + y0
z =  cos(q0) · r + z0
L1 = L2 = 133.35 mm,  (x0, y0, z0) = (0.0, 23.3, -168.3) mm   (OpenHaptics frame)
```

`L1 = L2 = 133.35 mm` matches the "Arm Length 133350" reported by the device itself.
The ROS frame is obtained as `x_ros = x`, `y_ros = −z`, `z_ros = y`, in metres.
Check: raw inkwell encoders `[0, 659, −910]` → (0.000, 0.088, −0.066) m, the same as
`/phantom/pose` in Deliverable 1.

The inverse kinematics is analytic: base angle from `atan2`, then a planar 2-link
problem (law of cosines).

## 2. Gravity model

### Model

With `q2` an absolute link angle, the gravity load on each motor is a combination of
sines and cosines of the joint angles, without constant terms:

```
g_i = c_i · [cos q1, sin q1, sin q2, cos q2]
```

`gravity_calibration.py` fits a *decoupled* variant (motor 1 uses q1 only, motor 2 uses
q2 only) and a *coupled* one, and keeps the coupled one only if it is clearly better.

### Procedure

```bash
cd tools/calibration
python3 gravity_calibration.py
```

1. Put the Touch at the table edge, so the stylus hangs freely.
2. Phase 1 (25 s, motors off): move the arm by hand through the whole workspace.
3. Phase 2 (automatic): the arm visits up to 16 poses chosen among those visited by
   hand; each pose is reached from both sides and the two holding commands are averaged,
   so that static friction cancels out.
4. The model is evaluated over the whole visited workspace and saved only if it never
   predicts commands above 550.

### Lessons learned (iterations)

| Version | Problem |
|---|---|
| polynomial on raw counts | fooled by static friction; many poses resting on the table |
| rectangular grid of joints | joints 1 and 2 have coupled limits: many poses unreachable (motor pushing a mechanical stop) |
| current | poses from the hand-visited workspace, 2-sided approach, physical model, safety check |

### Result on my device (decoupled model, 7 valid poses)

```
motor 1: -239.3 cos q1 - 78.4 sin q1     (RMS 101.8)
motor 2: +409.7 sin q2 + 252.1 cos q2     (RMS 50.2)
predicted over the workspace: motor 1 [-252, -25], motor 2 [+164, +481]
```

Poses with the second link nearly horizontal need more than 600 and were rejected:
full compensation everywhere is beyond what the motors can sustain, hence the partial
compensation (`gravity_gain` ≈ 0.7).
