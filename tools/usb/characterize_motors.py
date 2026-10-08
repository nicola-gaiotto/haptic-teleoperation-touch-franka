#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
characterize_motors.py - automatic characterisation of the 3 Touch motors over USB.

For each motor (0, 1, 2) and each direction (+delta, -delta) it runs:
  1.5 s zero current   -> mean encoders "before"
  1.5 s motor active   -> mean encoders "during" (last 0.4 s)
  1.5 s zero current   -> pause, so the arm comes to rest
At the end it prints the change of the 6 encoder values and, for each motor, which
encoder responds and with which sign (same or opposite to the command).

Result on the reference device (used as MOTOR_SIGN in touch_raw_driver):
  motor 0 -> enc0, opposite | motor 1 -> enc1, opposite | motor 2 -> enc2, same

USAGE (hands-off: nobody touches the Touch during the ~27 s test):
  python3 characterize_motors.py [delta]     default 400, maximum 400
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
MAX_DELTA = 400
PERIOD = 0.002
T_PHASE = 1.5


def packet(counter, cmd):
    return struct.pack('<BIHHHB', 0x0A, counter & 0xFFFFFFFF, cmd[0], cmd[1], cmd[2], FLAGS)


def mean(rows):
    if not rows:
        return [float('nan')] * 6
    return [sum(r[i] for r in rows) / len(rows) for i in range(6)]


def main():
    delta = int(sys.argv[1]) if len(sys.argv) > 1 else 400
    delta = max(1, min(MAX_DELTA, abs(delta)))

    tests = [(m, s * delta) for m in (0, 1, 2) for s in (+1, -1)]

    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit('Touch not found.')
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except (usb.core.USBError, NotImplementedError):
        pass
    usb.util.claim_interface(dev, 0)

    print(f'{len(tests)} tests, about {len(tests) * 3 * T_PHASE:.0f} s in total. Do NOT touch the Touch.')
    counter = 0
    results = []
    try:
        for (motor, d) in tests:
            print(f'  test: motor {motor}, delta {d:+d} ...', flush=True)
            before, during = [], []
            t0 = time.monotonic()
            while True:
                t = time.monotonic() - t0
                if t >= 3 * T_PHASE:
                    break
                active = T_PHASE <= t < 2 * T_PHASE
                cmd = [ZERO, ZERO, ZERO]
                if active:
                    cmd[motor] = ZERO + d
                dev.write(EP_OUT, packet(counter, cmd), timeout=100)
                counter += 1
                try:
                    data = bytes(dev.read(EP_IN, 64, timeout=20))
                except usb.core.USBTimeoutError:
                    data = b''
                if len(data) >= 35 and data[0] == 0x0B:
                    enc = struct.unpack('<13h', data[9:35])[:6]
                    if T_PHASE - 0.5 <= t < T_PHASE:
                        before.append(enc)
                    elif 2 * T_PHASE - 0.4 <= t < 2 * T_PHASE:
                        during.append(enc)
                time.sleep(PERIOD)
            b, du = mean(before), mean(during)
            results.append((motor, d, [x - y for x, y in zip(du, b)]))
    except KeyboardInterrupt:
        print('Interrupted.')
    finally:
        for _ in range(50):
            try:
                dev.write(EP_OUT, packet(counter, [ZERO, ZERO, ZERO]), timeout=100)
            except usb.core.USBError:
                pass
            counter += 1
            time.sleep(PERIOD)
        usb.util.release_interface(dev, 0)
        usb.util.dispose_resources(dev)

    print('\nEncoder change (during - before):')
    print('motor  delta |   enc0    enc1    enc2    enc3    enc4    enc5')
    for motor, d, dv in results:
        print(f'   {motor}   {d:+5d} | ' + ' '.join(f'{x:+7.0f}' for x in dv))

    print('\nSummary (encoder with the largest change among the first 3):')
    for motor in (0, 1, 2):
        rows = [(d, dv) for (m, d, dv) in results if m == motor]
        best = max(rows, key=lambda r: max(abs(v) for v in r[1][:3]))
        d, dv = best
        idx = max(range(3), key=lambda i: abs(dv[i]))
        same = (dv[idx] > 0) == (d > 0)
        print(f'  motor {motor}: encoder {idx}, change {dv[idx]:+.0f} with delta {d:+d} '
              f'-> sign {"SAME" if same else "OPPOSITE"}')
    print('\nZero current sent, interface released.')


if __name__ == '__main__':
    main()
