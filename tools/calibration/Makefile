# Build the kinematic calibration logger (needs OpenHaptics installed: libHD in /usr/lib).
# The warning "'stdcall' attribute ignored" comes from the OpenHaptics headers and is harmless.
kin_logger: kin_logger.c
	gcc -o kin_logger kin_logger.c -lHD -lrt -lm -lpthread

clean:
	rm -f kin_logger

.PHONY: clean
