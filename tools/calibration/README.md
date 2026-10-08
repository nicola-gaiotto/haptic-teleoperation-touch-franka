# Calibration tools

| File | Purpose |
|---|---|
| `kin_logger.c` + `Makefile` | Records encoders, joint angles and position with OpenHaptics (read-only, no forces). Build with `make`. |
| `kin_fit.py` | Fits encoders → angles → position and writes `~/.touch_raw_driver/kinematics.json`. |
| `gravity_calibration.py` | Identifies the gravity model over USB and writes `~/.touch_raw_driver/gravity.json`. |

Procedures and results: [docs/calibration.md](../../docs/calibration.md).

After a successful calibration, copy the JSON files into `calibration/` of this
repository so they are versioned:

```bash
cp ~/.touch_raw_driver/*.json ~/geomagic_ws/src/haptic-teleoperation-touch-franka/calibration/
```
