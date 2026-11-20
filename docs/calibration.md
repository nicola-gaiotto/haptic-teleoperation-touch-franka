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
gcc -o kin_logger kin_logger.c -lHD -lrt -lm -lpthread   # needs OpenHaptics
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

## 3. Friction

`tools/calibration/friction_identification.py` holds all joints with a PID, then slowly
increases the command of one joint, in each direction, until the joint breaks away (three
repetitions, median). For joints 1 and 2 the two breakaway commands lie around the gravity
load, so the friction is half of their difference.

Result on my device (command units):

| Joint | Breakaway friction | Remark |
|---|---|---|
| 0 (base) | 168 in one direction, 194 in the other | left/right asymmetry |
| 1 | 172 | gravity load about 285 |
| 2 | 187 | gravity load about 240 |

50 % of these values is compensated in the driver (`friction_pos`, `friction_neg` in
`config/touch_raw_driver.yaml`).

## 4. Force mode: motor command ↔ newton (`force_gain`)

`tools/calibration/force_calibration.py`, lab PC, 2026-11-20, kitchen scale (1 g resolution).

Setup: the driver in `force` mode; the stylus **lying flat** on the scale plate under the
gimbal (the point of `/touch/pose`), so the arm pushes down through it (the stylus cannot stand
on its tip). Scale tared with zero force. The script requests downward forces and the reading
in grams is typed in.

Joint friction holds part of the load while the arm is still: with forces only increasing
(0.25–1.25 N requested) the scale read 0 g up to 0.75 N, so that run was discarded. The final
procedure goes **up and then down** without returning to zero (0.5 → 2.5 → 0.5 N, two passes):
going up the scale reads about `k F − f`, going down `k F + f`, and the mean cancels `f`.

| Requested [N] (gain 2000) | Up [g] (pass 1 / 2) | Down [g] (pass 1 / 2) | Mean [N] |
|---|---|---|---|
| 0.5 | 0 / 7 | 26 / 25 | 0.142 |
| 1.0 | 11 / 16 | 38 / 38 | 0.253 |
| 1.5 | 31 / 34 | 56 / 57 | 0.437 |
| 2.0 | 50 / 52 | 69 / 69 | 0.589 |
| 2.5 | 73 / 72 (top) | | 0.711 |

Fit: measured = 0.295 × requested − 0.016 N (RMS residual 0.016 N) → `force_gain` =
2000 / 0.295 ≈ **6780 command per N m**. Friction seen at the tip: about ±0.12 N.

Check with `force_gain` 6780 (0.2 → 0.6 → 0.2 N, two passes): 0.2 N → 0.194 N, 0.4 N → 0.385 N
(mean up/down). The 0.6 N point has only "up" readings (0.52 N, i.e. minus the friction).

Consequences:
- The motor command limit (600) corresponds to about **0.7 N** at the tip in this pose, so the
  Touch can render at most about 0.7 N (the 3D Systems specification gives 3.3 N peak, which
  is not reached with this USB command range).
- Only the vertical direction (joints 1 and 2) was calibrated; joint 0 is assumed to have the
  same gain *(hypothesis)*.
- Before this calibration (2026-11-17/19), forces in "N" sent to the Touch were about 0.3 × the
  real ones.
