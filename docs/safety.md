# Safety

The direct-USB driver bypasses the protections of OpenHaptics, so it implements its own.

## Emergency stop

1. **Ctrl+C** in the terminal of `touch_raw_driver`: the node sends zero current before
   exiting.
2. **Unplug the power supply** of the Touch: the motors stop immediately.

Never kill the node with `kill -9`: the zero-current shutdown would be skipped.

## Built-in protections (`touch_raw_driver`)

| Protection | Behaviour |
|---|---|
| Disabled at start | zero current until `/touch/enable` is set to true |
| No jump at enable | the target is set to the current position |
| Speed-limited reference | the reference moves towards the target at `max_ref_speed` |
| Saturation | motor command limited to `max_delta` (hard limit 600 in the code) |
| Workspace limits | Cartesian targets are converted to joint targets clamped inside the recorded joint ranges (5 % margin) |
| Stale data | zero current if no valid state report is received |
| USB error | the node disables itself and sends zero current |
| Thermal protection | if the RMS command of a motor over `rms_time_constant` exceeds `rms_limit`, the motor is capped at 90 % of the limit until its RMS falls below 85 % (hysteresis) |
| Integral anti-windup | the integral is frozen while the command is saturated |

## Good practice

- Start with the stylus in the inkwell; keep a hand close to the stylus when enabling.
- When disabling, hold the stylus: with zero current the arm drops.
- Change one gain at a time and observe the result before the next change.
- The thermal limits are conservative choices (no vendor temperature model is available
  for the direct driver): do not raise them for long sessions.
- Calibration scripts move the arm automatically: keep the area around the device free.
