#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
"""
raw_motor_test.py - first direct test of the Touch motors over USB, WITHOUT the
3D Systems libraries.

It uses the protocol decoded with usbmon (see docs/usb_protocol.md):
  PC -> Touch, EP 0x02, report 10 (12 bytes):
      0x0a | counter uint32 | motor0 uint16 | motor1 uint16 | motor2 uint16 | flags 0x03
      (2048 = 0x0800 = zero current)
  Touch -> PC, EP 0x81, report 11 (35 bytes):
      0x0b | counter uint32 | counter echo uint32 | 13 int16 values (encoders, ...)

Usage:
  python3 raw_motor_test.py [motor 0|1|2] [delta]
  default: motor 0, delta 150 (limited to +/-400 around 2048)

Sequence: 2 s zero current, 2 s with 2048+delta on the chosen motor, 2 s zero current.
Every 0.25 s it prints: phase, command, counter echo, first 6 values (encoders).
Zero current is always sent at the end and on Ctrl+C.
For a clear result, do NOT touch the device ("hands-off" test).
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
PERIOD = 0.002  # s, same rate as the official library (about 500 Hz)


def packet(counter, m0, m1, m2):
    return struct.pack('<BIHHHB', 0x0A, counter & 0xFFFFFFFF, m0, m1, m2, FLAGS)


def main():
    motor = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    delta = int(sys.argv[2]) if len(sys.argv) > 2 else 150
    if motor not in (0, 1, 2):
        sys.exit('The motor must be 0, 1 or 2.')
    delta = max(-MAX_DELTA, min(MAX_DELTA, delta))

    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        sys.exit('Touch not found (check cable, power supply and lsusb).')
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except (usb.core.USBError, NotImplementedError):
        pass
    usb.util.claim_interface(dev, 0)

    print(f'Motor {motor}, command {ZERO} -> {ZERO + delta} (delta {delta}).')
    print('Phases: 0-2 s zero | 2-4 s ACTIVE | 4-6 s zero. Do not touch the device.')

    counter = 0
    last_print = -1.0
    t0 = time.monotonic()
    try:
        while True:
            t = time.monotonic() - t0
            if t >= 6.0:
                break
            active = 2.0 <= t < 4.0
            cmd = [ZERO, ZERO, ZERO]
            if active:
                cmd[motor] = ZERO + delta
            dev.write(EP_OUT, packet(counter, *cmd), timeout=100)
            counter += 1

            try:
                data = bytes(dev.read(EP_IN, 64, timeout=20))
            except usb.core.USBTimeoutError:
                data = b''

            if t - last_print >= 0.25 and len(data) >= 35 and data[0] == 0x0B:
                echo = struct.unpack('<I', data[5:9])[0]
                vals = struct.unpack('<13h', data[9:35])
                phase = 'ACTIVE' if active else 'zero  '
                print(f't={t:4.2f} s | {phase} | cmd={cmd} | echo={echo} | enc={list(vals[:6])}')
                last_print = t
            time.sleep(PERIOD)
    except KeyboardInterrupt:
        print('Interrupted.')
    finally:
        for _ in range(50):
            try:
                dev.write(EP_OUT, packet(counter, ZERO, ZERO, ZERO), timeout=100)
            except usb.core.USBError:
                pass
            counter += 1
            time.sleep(PERIOD)
        usb.util.release_interface(dev, 0)
        usb.util.dispose_resources(dev)
        print('Zero current sent, interface released.')


if __name__ == '__main__':
    main()
