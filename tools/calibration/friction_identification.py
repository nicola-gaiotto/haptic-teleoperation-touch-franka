#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
friction_identification.py - measures the static (breakaway) friction of each Touch joint
in both directions, commanding the motors directly over USB (no OpenHaptics).

For each joint and each direction:
  1. all joints are held still by a PID for 2 s; the command that holds the tested joint
     (gravity load, ~0 for the base) is averaged;
  2. the PID of the tested joint is replaced by: hold command + a slow ramp in the tested
     direction (the other joints stay held by their PID);
  3. as soon as the joint moves by more than BREAKAWAY counts, the TOTAL command
     (hold + ramp) is recorded: the breakaway command in that direction;
  4. the PID brings the joint back to its start position.
Each measurement is repeated REPEATS times; medians are used.

Analysis: static friction also holds the joint during the hold phase, so the hold command
varies between repetitions; only the total breakaway command is meaningful. The two
breakaway commands (+ and -) lie symmetrically around the load of the joint (gravity;
~0 for the base), so:
    load     = (breakaway_plus + breakaway_minus) / 2
    friction = (breakaway_plus - breakaway_minus) / 2
For the base (no gravity) the two directions are reported separately.
The results are in the "u" convention of touch_raw_driver (positive u = encoder count
increasing), ready for its parameters friction_pos / friction_neg.

USAGE (hands-off; put the stylus at mid-workspace before the countdown, then let go):
  python3 friction_identification.py [joints]      default: 0 1 2, e.g. "0" for the base only
Emergency stop: Ctrl+C (zero current is always sent), or unplug the Touch power supply.
"""
import struct
import sys
import time

import usb.core
import usb.util

VID, PID = 0x2988, 0x0304
EP_OUT, EP_IN = 0x02, 0x81
ZERO = 0x0800
FLAGS = 0x03
MOTOR_SIGN = (-1, -1, +1)          # measured with tools/usb/characterize_motors.py
MAX_DELTA = (400, 600, 600)

KP = (0.6, 0.8, 0.8)
KI = (1.0, 3.0, 3.0)
KD = (0.03, 0.03, 0.03)
INTEGRAL_LIMIT = 500.0
VEL_ALPHA = 0.2

HOLD_TIME = 2.0                    # s
RAMP_RATE = 60.0                   # command units per second
RAMP_MAX = 350.0                   # stop the ramp here (no breakaway = friction > RAMP_MAX)
BREAKAWAY = 25                     # counts
RETURN_TIME = 2.5                  # s
REPEATS = 3


class TouchUSB:
    def __init__(self):
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            sys.exit('Touch not found.')
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except (usb.core.USBError, NotImplementedError):
            pass
        usb.util.claim_interface(dev, 0)
        self.dev = dev
        self.counter = 0

    def exchange(self, deltas):
        cmd = [ZERO + int(d) for d in deltas]
        pkt = struct.pack('<BIHHHB', 0x0A, self.counter & 0xFFFFFFFF,
                          cmd[0], cmd[1], cmd[2], FLAGS)
        self.dev.write(EP_OUT, pkt, timeout=100)
        self.counter += 1
        try:
            data = bytes(self.dev.read(EP_IN, 64, timeout=20))
        except usb.core.USBTimeoutError:
            return None
        if len(data) >= 35 and data[0] == 0x0B:
            return struct.unpack('<13h', data[9:35])[:3]
        return None

    def send_zero(self, n=50):
        for _ in range(n):
            try:
                self.exchange([0, 0, 0])
            except usb.core.USBError:
                pass
            time.sleep(0.002)

    def close(self):
        usb.util.release_interface(self.dev, 0)
        usb.util.dispose_resources(self.dev)


class Controller:
    """Joint PID in the u convention; joint `override` gets an external u instead."""

    def __init__(self, touch):
        self.touch = touch
        enc = None
        while enc is None:
            enc = touch.exchange([0, 0, 0])
        self.enc = list(enc)
        self.target = [float(e) for e in enc]
        self.integ = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.u = [0.0, 0.0, 0.0]
        self.deltas = [0, 0, 0]
        self.last_t = time.monotonic()

    def tick(self, override=None, u_override=0.0):
        now = time.monotonic()
        dt = max(now - self.last_t, 1e-4)
        self.last_t = now
        enc = self.touch.exchange(self.deltas)
        if enc is None:
            return now
        for i in range(3):
            raw_v = (enc[i] - self.enc[i]) / dt
            self.vel[i] = VEL_ALPHA * raw_v + (1 - VEL_ALPHA) * self.vel[i]
        self.enc = list(enc)
        for i in range(3):
            if i == override:
                u = u_override
            else:
                err = self.target[i] - enc[i]
                self.integ[i] = max(-INTEGRAL_LIMIT,
                                    min(INTEGRAL_LIMIT, self.integ[i] + KI[i] * err * dt))
                u = KP[i] * err + self.integ[i] - KD[i] * self.vel[i]
            self.u[i] = u
            d = MOTOR_SIGN[i] * u
            self.deltas[i] = int(max(-MAX_DELTA[i], min(MAX_DELTA[i], d)))
        time.sleep(0.001)
        return now

    def hold(self, seconds, joint=None):
        """Hold all joints; return the mean u of `joint` over the last second."""
        t0 = time.monotonic()
        samples = []
        while True:
            now = self.tick()
            if now - t0 > seconds:
                break
            if joint is not None and now - t0 > seconds - 1.0:
                samples.append(self.u[joint])
        return sum(samples) / len(samples) if samples else 0.0

    def ramp(self, joint, hold_u, direction):
        """Ramp u of `joint` from hold_u until breakaway. Returns the ramp value or None."""
        start = self.enc[joint]
        t0 = time.monotonic()
        while True:
            r = RAMP_RATE * (time.monotonic() - t0)
            if r > RAMP_MAX:
                return None
            self.tick(override=joint, u_override=hold_u + direction * r)
            if abs(self.enc[joint] - start) > BREAKAWAY:
                return r


def median(values):
    v = sorted(values)
    return v[len(v) // 2] if v else None


def main():
    joints = [int(a) for a in sys.argv[1:]] or [0, 1, 2]
    touch = TouchUSB()
    results = {}
    try:
        print('Hold the stylus at mid-workspace. Motors ON in:')
        for k in (3, 2, 1):
            print(f'  {k}...', flush=True)
            t_end = time.monotonic() + 1.0
            while time.monotonic() < t_end:
                touch.exchange([0, 0, 0])
                time.sleep(0.002)
        ctrl = Controller(touch)
        home = list(ctrl.target)
        print('Motors ON. Let go of the stylus (keep your hand close).')
        for j in joints:
            for direction in (+1, -1):
                values = []
                for rep in range(REPEATS):
                    ctrl.target = [float(h) for h in home]
                    hold_u = ctrl.hold(HOLD_TIME, joint=j)
                    r = ctrl.ramp(j, hold_u, direction)
                    ctrl.target = [float(h) for h in home]
                    ctrl.integ[j] = hold_u      # restart the PID from the hold command
                    ctrl.hold(RETURN_TIME)
                    if r is None:
                        label = 'no breakaway'
                    else:
                        values.append(hold_u + direction * r)
                        label = f'breakaway at u = {hold_u + direction * r:+6.0f}'
                    print(f'  joint {j} {"+" if direction > 0 else "-"} (rep {rep + 1}): '
                          f'hold u = {hold_u:+6.0f}, {label}')
                results[(j, direction)] = median(values)
        touch.send_zero()

        print('\nRESULT (u convention of touch_raw_driver):')
        fpos, fneg = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        for j in joints:
            bp, bm = results.get((j, +1)), results.get((j, -1))
            if bp is None or bm is None:
                print(f'  joint {j}: incomplete measurement, not used')
                continue
            if j == 0:      # no gravity on the base: keep the two directions separate
                fpos[j], fneg[j] = max(bp, 0.0), max(-bm, 0.0)
                print(f'  joint 0: breakaway +{bp:.0f} / {bm:.0f} -> '
                      f'friction_pos {fpos[0]:.0f}, friction_neg {fneg[0]:.0f}')
            else:
                f = max((bp - bm) / 2.0, 0.0)
                fpos[j] = fneg[j] = f
                print(f'  joint {j}: breakaway {bp:+.0f} / {bm:+.0f} -> load {(bp + bm) / 2:+.0f}, '
                      f'friction {f:.0f}')
        fmt = lambda v, k: '[' + ', '.join(f'{k * x:.1f}' for x in v) + ']'
        print('\nStart with 50 % of the friction (raise towards 70 % if stable):')
        print(f'  ros2 param set /touch_raw_driver friction_pos "{fmt(fpos, 0.5)}"')
        print(f'  ros2 param set /touch_raw_driver friction_neg "{fmt(fneg, 0.5)}"')
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        touch.send_zero()
        touch.close()
        print('Zero current sent, interface released.')


if __name__ == '__main__':
    main()
