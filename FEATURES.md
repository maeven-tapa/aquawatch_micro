Open `aquawatch_build/aquawatch_build.ino` in Arduino IDE and select NodeMCU 1.0 (ESP-12E Module). This is a synchronized copy of `aquawatch.ino`, in its own sketch folder so the reference sketches are not compiled with it.

Keep `aquawatch_logo.h` beside the sketch. It embeds the supplied logo in flash; no filesystem upload or internet connection is required. The dashboard uses orange accents, a three-column rotation/acceleration row, and a contact form hidden until the + button is pressed.

The header reads **AquaWatch Access Control**. The logo is served as a transparent PNG without a frame. The prominent indicator below shows **Alarm: ACTIVE/OFF**; connection errors still appear when updates fail, while the routine connection status stays hidden.

Device name defaults to **Spike**. Select **Edit** beside the displayed name to reveal the input, then **Save** or **Cancel**. Names accept up to 16 letters/numbers/spaces/dots/underscores/hyphens and persist in EEPROM separately from the contact list. Every SMS blast includes `Device: <name>`; changes take effect on the next blast snapshot.

- GPS: expand **View raw GPS data (NMEA)** for the latest 1.5 KB of incoming GPS text and its age. This works before a position fix is available.
- SIM800L: `AT+CSQ` is queried every 10 seconds while the modem is free. SMS transactions take priority. Missing or older-than-30-second readings show unavailable/stale; CSQ 99 shows unknown. Signal strength is in dBm, not dBi. Conversion follows the [SIMCom AT command manual](https://files.waveshare.com/upload/2/26/SIM800_Series_AT_Command_Manual_V1.11.pdf), including the bounded CSQ 0 and 31 readings.
- Automatic SOS: either X or Y reaching +50 degrees or -50 degrees starts the siren, strobe, and SMS blast. These are the integrated, calibrated rotations from `gyro.ino`, relative to startup; they are not absolute gravity-referenced tilt angles. Keep the device still during startup calibration.
- Cancelling by the button or web control suppresses automatic retriggering until both axes return strictly inside +/-45 degrees. Returning upright does not cancel an active alarm.
- GPS, sensor, button and dashboard servicing continue during modem waits. Contacts receive a common snapshot in sequence; an SMS already submitted cannot be recalled by cancelling. The next blast is due 60 seconds after the previous start, without overlapping blasts.

Validation: compile for `esp8266:esp8266:nodemcuv2`; `node check_features.cjs` checks alarm boundaries and dashboard rendering with mocked inputs. Actual GPS, modem signal, SMS reception, and relay timing require a connected hardware test.

Dashboard recovery (1.3.1): responses escape all JSON control bytes. Page/API caching is disabled. Dashboard polling uses one timed request at a time, displays parsing/network errors and retries automatically. A status line distinguishes BMI160 read failures from GPS receiving bytes but awaiting a fix. After uploading, reconnect to `aquawatch_dev` and reload `http://192.168.4.1`; the alarm indicator should update to `Alarm: OFF` or `Alarm: ACTIVE`. If it still fails, the visible error and the response at `http://192.168.4.1/api` identify the next diagnostic step. The user's exact device-side failure was not reproduced remotely.
