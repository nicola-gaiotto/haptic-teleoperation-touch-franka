# Calibration files of my device

Device: 3D Systems Touch, serial 24079000373.

| File | Content | How it was obtained |
|---|---|---|
| `kinematics.json` | encoders → joint angles → position | `tools/calibration/kin_logger` + `kin_fit.py` |
| `gravity.json` | gravity compensation model | `tools/calibration/gravity_calibration.py` |

These values are **specific to this device**. With another Touch, run the calibration
again (see [docs/calibration.md](../docs/calibration.md)).

The driver reads them from `~/.touch_raw_driver/`:

```bash
mkdir -p ~/.touch_raw_driver && cp *.json ~/.touch_raw_driver/
```
