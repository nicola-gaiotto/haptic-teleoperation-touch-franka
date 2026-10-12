#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
raw_state_monitor.py - prints all 13 int16 values of the Touch state report (report 11),
with zero current on the motors. Used to find where the stylus buttons are encoded.

Usage:
  python3 raw_state_monitor.py [seconds]        default 30 s

Values 0..5 are the encoders (arm + gimbal). Values 6..12 are status values whose
meaning is not documented: press and release the two stylus buttons and watch which
value changes (marked with '*').
Close every other program that uses the Touch first.
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


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit('Touch not found.')
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except (usb.core.USBError, NotImplementedError):
        pass
    usb.util.claim_interface(dev, 0)

    print('Press and release the GREY button, then the WHITE button, then both.')
    print('            ' + ' '.join(f'{i:>6d}' for i in range(13)))
    counter = 0
    last = None
    last_print = 0.0
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < duration:
            pkt = struct.pack('<BIHHHB', 0x0A, counter & 0xFFFFFFFF, ZERO, ZERO, ZERO, FLAGS)
            dev.write(EP_OUT, pkt, timeout=100)
            counter += 1
            try:
                data = bytes(dev.read(EP_IN, 64, timeout=20))
            except usb.core.USBTimeoutError:
                continue
            if len(data) < 35 or data[0] != 0x0B:
                continue
            vals = struct.unpack('<13h', data[9:35])
            now = time.monotonic()
            # print on any change of the status values (6..12), or every 0.5 s
            status_changed = last is not None and vals[6:] != last[6:]
            if status_changed or now - last_print > 0.5:
                marks = ['*' if last is not None and i >= 6 and vals[i] != last[i] else ' '
                         for i in range(13)]
                print(f't={now - t0:5.1f} s ' + ' '.join(f'{v:>5d}{m}' for v, m in zip(vals, marks)))
                last_print = now
            last = vals
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(20):
            try:
                dev.write(EP_OUT, struct.pack('<BIHHHB', 0x0A, counter, ZERO, ZERO, ZERO, FLAGS),
                          timeout=100)
            except usb.core.USBError:
                pass
            counter += 1
            time.sleep(0.002)
        usb.util.release_interface(dev, 0)
        usb.util.dispose_resources(dev)
        print('Done, interface released.')


if __name__ == '__main__':
    main()
