// SPDX-License-Identifier: MIT
// Part of haptic-teleoperation-touch-franka - Copyright (c) 2026 Nicola Gaiotto
/*
 * kin_logger.c - kinematic calibration logger for the 3D Systems Touch.
 *
 * Uses OpenHaptics ONLY TO READ (position reading works on this system; forces are
 * disabled). At 100 Hz it records:
 *   - raw encoder values        HD_CURRENT_ENCODER_VALUES  (6 values)
 *   - joint angles [rad]        HD_CURRENT_JOINT_ANGLES    (3 values)
 *   - gimbal angles [rad]       HD_CURRENT_GIMBAL_ANGLES   (3 values)
 *   - tip position [mm]         HD_CURRENT_POSITION        (OpenHaptics frame)
 * The CSV is then used by kin_fit.py to map raw encoder counts (what touch_raw_driver
 * reads over USB) to joint angles and to the Cartesian position of the stylus.
 *
 * Usage:   ./kin_logger [seconds] [output.csv]      default: 60 s, kin_log.csv
 * Build:   gcc -o kin_logger kin_logger.c -lHD -lrt -lm -lpthread
 *
 * Procedure: keep the stylus IN THE INKWELL for the first 5 s, then move it slowly
 * through the whole workspace (rotate the base left/right, up/down, near/far).
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include <HD/hd.h>

typedef struct {
    HDlong enc[6];
    HDdouble joints[3];
    HDdouble gimbal[3];
    HDdouble pos[3];
} Sample;

/* Runs in the haptic thread: reads one consistent snapshot of the device state. */
static HDCallbackCode HDCALLBACK sample_callback(void *data)
{
    Sample *s = (Sample *)data;
    hdBeginFrame(hdGetCurrentDevice());
    hdGetLongv(HD_CURRENT_ENCODER_VALUES, s->enc);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, s->joints);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, s->gimbal);
    hdGetDoublev(HD_CURRENT_POSITION, s->pos);
    hdEndFrame(hdGetCurrentDevice());
    return HD_CALLBACK_DONE;
}

static double now_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static int check_error(const char *where)
{
    HDErrorInfo err = hdGetError();
    if (HD_DEVICE_ERROR(err)) {
        fprintf(stderr, "OpenHaptics error in %s: code 0x%04x, internal %d\n",
                where, err.errorCode, err.internalErrorCode);
        return 1;
    }
    return 0;
}

int main(int argc, char *argv[])
{
    double duration = (argc > 1) ? atof(argv[1]) : 60.0;
    const char *path = (argc > 2) ? argv[2] : "kin_log.csv";
    Sample s = {0};

    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (check_error("hdInitDevice"))
        return 1;

    hdDisable(HD_FORCE_OUTPUT);   /* read-only: never command forces */
    hdStartScheduler();
    if (check_error("hdStartScheduler")) {
        hdDisableDevice(hHD);
        return 1;
    }

    FILE *f = fopen(path, "w");
    if (!f) {
        perror("fopen");
        hdStopScheduler();
        hdDisableDevice(hHD);
        return 1;
    }
    fprintf(f, "t,enc0,enc1,enc2,enc3,enc4,enc5,q0,q1,q2,g0,g1,g2,x,y,z\n");

    printf("Logging %.0f s to %s.\n", duration, path);
    printf("Keep the stylus IN THE INKWELL for the first 5 s, then move it slowly\n");
    printf("through the whole workspace (base left/right, up/down, near/far).\n\n");

    double t0 = now_s(), next_print = 0.0;
    long n = 0;
    while (1) {
        double t = now_s() - t0;
        if (t >= duration)
            break;
        hdScheduleSynchronous(sample_callback, &s, HD_DEFAULT_SCHEDULER_PRIORITY);
        fprintf(f, "%.4f,%ld,%ld,%ld,%ld,%ld,%ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f\n",
                t, (long)s.enc[0], (long)s.enc[1], (long)s.enc[2],
                (long)s.enc[3], (long)s.enc[4], (long)s.enc[5],
                s.joints[0], s.joints[1], s.joints[2],
                s.gimbal[0], s.gimbal[1], s.gimbal[2],
                s.pos[0], s.pos[1], s.pos[2]);
        n++;
        if (t >= next_print) {
            const char *phase = (t < 5.0) ? "INKWELL" : "MOVE   ";
            printf("  %5.1f s left | %s | enc=[%ld %ld %ld] | pos=(%7.1f %7.1f %7.1f) mm\n",
                   duration - t, phase, (long)s.enc[0], (long)s.enc[1], (long)s.enc[2],
                   s.pos[0], s.pos[1], s.pos[2]);
            fflush(stdout);
            next_print += 1.0;
        }
        usleep(10000);   /* about 100 Hz */
    }

    fclose(f);
    hdStopScheduler();
    hdDisableDevice(hHD);
    printf("\nDone: %ld samples written to %s\n", n, path);
    return 0;
}
