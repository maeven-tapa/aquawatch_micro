# Tilt regression checks

`test_tilt.cpp` executes the same `tilt_estimator.h` used by the firmware, with
synthetic accelerometer and gyro readings. No Arduino or sensor mock replaces
the estimator itself.

From the sketch directory, in a Visual Studio Developer Command Prompt:

```bat
cl /nologo /EHsc /std:c++14 /W4 tests\test_tilt.cpp /Fe:"%TEMP%\aquawatch-test-tilt.exe" /Fo:"%TEMP%\aquawatch-test-tilt.obj"
"%TEMP%\aquawatch-test-tilt.exe"
```

Checks cover five mounting orientations, both axes, repeated +/-60-degree
movement, sustained positive and negative tilts, returning upright, ten minutes
of residual gyro bias, angles beyond the 90-degree alarm threshold, long sample
gaps, large acceleration rejection, invalid samples, yaw, and inversion.

Calibration retry and dashboard checks:

```bat
cl /nologo /EHsc /std:c++14 /W4 tests\test_calibration.cpp /Fe:"%TEMP%\aquawatch-test-calibration.exe" /Fo:"%TEMP%\aquawatch-test-calibration.obj"
"%TEMP%\aquawatch-test-calibration.exe"
node tests\test_dashboard_imu.cjs
```

Calibration tests cover all six axis-aligned startup poses, automatic recovery
after a rejected moving window, high gyro rate, invalid acceleration, missing
samples, timer wraparound, and preserving the reference after calibration.
Dashboard checks run the actual embedded script with a mocked DOM.

## App status checks

From the sketch directory in a Visual Studio Developer Command Prompt:

```bat
powershell -NoProfile -ExecutionPolicy Bypass -File tests\run_app_status_tests.ps1
```

The harness extracts the actual app-status and alarm functions from the sketch
and tests them with a mock modem and clock. Checks cover startup without
distress or GPS, the three-minute interval, registration retries, SOS priority
between recipients, alarm cancellation, alarm-only operation, failed sends,
empty contacts, modem contention, and timer wraparound.

App contacts receive a status batch once GSM is registered after startup,
then three minutes after each batch finishes while the device stays powered.
Alarm changes do not reset this timer. SOS contacts still receive distress
alerts only. Registration checks retry every ten seconds when unavailable;
these checks do not send texts.

## Device check after upload

Keep `tilt_estimator.h`, `imu_calibration.h`, and `app_status_schedule.h` beside
`aquawatch_build.ino`.
Keep the assembled device still in its normal operating position at startup;
the BMI160 board itself can be flat, vertical, or face down. Calibration runs
alongside modem initialization and waits for about one second of stable sensor
readings. Rejected windows retry automatically without stopping I2C reads.
Once successful, calibration stays fixed until reboot; normal tilt or later
read failures do not reset the reference.

The dashboard distinguishes initialization failure, I2C read timeout,
calibration in progress (with reason), and tilt readiness. Expand **BMI160
diagnostics** to see raw gyro/acceleration readings, sample age and error count.
If vertical startup still cannot calibrate, these values distinguish movement,
an unexpected acceleration magnitude, and an actual communication failure.

1. Confirm both readings are close to zero after startup.
2. Tilt one direction about 30-45 degrees and hold for 20 seconds. Its reading
   should remain near that tilt, rather than falling toward zero.
3. Return to the starting pose; the reading should settle near zero. Repeat in
   the opposite direction and on the other axis, then repeat several cycles.
4. Check the actual boat mounting and both sides of 90 degrees before relying
   on the automatic distress trigger. Existing alarm thresholds and timing are
   unchanged.

The readings are inclinations in two planes, measured against gravity relative
to the startup pose. They are not accumulated turns or compass heading. Turning
around the vertical while level should not create a tilt alarm. The reference
Y direction follows the sensor Y axis projected onto the upright horizontal
plane; if sensor Y points nearly vertically, sensor -Z is used instead. Axis X
completes the perpendicular frame. The sensor must remain fixed to the device.

For inclinations beyond 90 degrees, both plane readings can be large. They are
not a conventional Euler roll/pitch pair. Sustained translational acceleration
near 1 g can still affect a gravity-based tilt estimate; simulation does not
replace checks with the actual sensor and boat motion.

Background: [Analog Devices AN-1057, accelerometer inclination sensing](https://www.analog.com/en/resources/app-notes/an-1057.html).
