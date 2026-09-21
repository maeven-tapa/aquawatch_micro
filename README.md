<div align="center">
  <img src="aquawatch-logo.png" alt="AquaWatch logo with an orange fish-shaped eye and white lettering" width="760">
  <h1>AquaWatch Micro</h1>
  <p><strong>Motion monitoring and SOS alerts for NodeMCU.</strong></p>
  <p>Live rotation and GPS readings, local Wi-Fi controls, and SMS distress messages.</p>
  <p>Motion sensing · GPS location · SMS alerts · Siren and strobe</p>
  <p>
    <img alt="ESP8266 NodeMCU" src="https://img.shields.io/badge/Board-ESP8266_NodeMCU-FF863E">
    <img alt="Arduino C++" src="https://img.shields.io/badge/Firmware-Arduino_C%2B%2B-00979D?logo=arduino&amp;logoColor=white">
    <img alt="BMI160 motion sensor" src="https://img.shields.io/badge/Motion-BMI160-54D8E8">
    <img alt="Development prototype" src="https://img.shields.io/badge/Status-Development_Prototype-FF6B00">
  </p>
  <p><a href="#overview">Overview</a> · <a href="#features">Features</a> · <a href="#wiring">Wiring</a> · <a href="#quick-start">Quick start</a> · <a href="#development-status">Development status</a></p>
</div>

## Overview

AquaWatch Micro is ESP8266 firmware that combines a BMI160 motion sensor, NEO-8M GPS receiver, SIM800L modem, and siren/strobe relays. The device hosts **AquaWatch Access Control**, a local dashboard available through its own Wi-Fi network.

An SOS can be started with the physical button, the dashboard, or the automatic rotation threshold. It activates the siren and blinking strobe and sends a snapshot of location and motion readings to the saved SMS contacts.

This is a **development prototype**. Compilation and simulated checks have passed; real sensor reception, cellular delivery, relay behavior, and operation under load still require hardware verification.

## Features

| Area | What it does |
| --- | --- |
| Motion dashboard | Displays X rotation, Y rotation, and estimated acceleration in three columns, using the calculations from `gyro.ino`. |
| GPS | Displays latitude/longitude, satellite count, fix age, a map link, and an expandable raw NMEA view. |
| SIM800L | Displays modem identity, reported status, signal strength in dBm, and SMS progress. |
| Manual SOS | Hold the button for 1.5 seconds to activate; release, then hold for 3 seconds to cancel. Dashboard controls are also available. |
| Automatic SOS | Either axis reaching +50° or −50° activates the alarm. After cancellation, both must return strictly inside ±45° before automatic triggering re-arms. |
| SMS blast | Sends to up to six saved contacts sequentially; repeats while the alarm remains active. |
| Device name | Defaults to **Spike**. Select **Edit** to reveal the name field; the saved name is included in SMS messages. |
| Contact management | The **+** button reveals the name/number form. Contact names and numbers persist after reboot. |
| Local interface | Orange accents, a transparent logo, alarm status, uptime, connection-error reporting, and automatic update retries. |

## Hardware

- NodeMCU ESP8266 development board.
- BMI160 accelerometer/gyroscope module.
- NEO-8M GPS receiver and antenna.
- SIM800L, antenna, and a SIM with SMS service on a compatible GSM/2G network.
- Active-high strobe relay and active-low siren relay.
- Push button connected to 3.3 V when pressed, with a pulldown on D0.
- Suitable power supplies for the modem, relays, and loads, with a shared signal ground.

## Wiring

| NodeMCU pin | GPIO | Connection |
| --- | --- | --- |
| D0 | 16 | SOS button, active high / pulldown |
| D1 | 5 | BMI160 SCL |
| D2 | 4 | BMI160 SDA |
| D3 | 0 | Strobe relay input, active high |
| D4 | 2 | Siren relay input, active low |
| D5 | 14 | GPS TX → NodeMCU RX |
| D6 | 12 | GPS RX ← NodeMCU TX |
| D7 | 13 | SIM800L TX → NodeMCU RX |
| D8 | 15 | SIM800L RX ← NodeMCU TX |

Both serial peripherals are configured at **9600 baud**. BMI160 initialization tries **0x69**, then **0x68**.

GPIO0 and GPIO2 must be high and GPIO15 low during normal boot. The active-high relay on D3 may activate before firmware initializes the pin; firmware cannot prevent that startup behavior. Check relay-module loading and boot levels before connecting loads. See the [ESP8266 board documentation](https://arduino-esp8266.readthedocs.io/en/3.0.0/boards.html).

Use a supply and logic-level interface appropriate to your particular SIM800L breakout. A bare SIM800 requires approximately 3.4–4.4 V and can draw roughly 2 A peaks; do not power it from the NodeMCU 3.3 V output. See the [SIMCom hardware design guide](https://simcom.ee/documents/SIM800/SIM800_Hardware%20Design_V1.09.pdf).

## Quick start

### 1. Install the tools and libraries

In Arduino IDE, add the following URL under **Additional Boards Manager URLs**:

```text
https://arduino.esp8266.com/stable/package_esp8266com_index.json
```

Install **esp8266 by ESP8266 Community** through Boards Manager. The project has been compiled with core **3.1.2**.

Install these libraries through Library Manager:

| Library | Version used for compilation |
| --- | --- |
| DFRobot_BMI160 | 1.0.0 |
| TinyGPSPlus | 1.0.3 |

Wire, EEPROM, ESP8266WiFi, ESP8266WebServer, and EspSoftwareSerial are supplied by the ESP8266 board package.

### 2. Open the upload sketch

Open **[`aquawatch_build/aquawatch_build.ino`](aquawatch_build/aquawatch_build.ino)**. Keep **`aquawatch_logo.h`** in that same directory.

Select **NodeMCU 1.0 (ESP-12E Module)** and the board's serial port, then upload. No filesystem upload is needed: the website and logo are embedded in firmware.

The root directory also contains reference sketches with their own `setup()` and `loop()` functions. Use the isolated `aquawatch_build` sketch directory to avoid compiling those references together.

### 3. Connect to the dashboard

1. Keep the device still during startup gyro calibration.
2. Connect your phone or computer to **aquawatch_dev**; there is no password.
3. Open **http://192.168.4.1**. If your phone reports no internet, stay connected to the device network.
4. Edit the device name if desired and configure the intended SMS recipients before activating SOS. The firmware contains an initial primary recipient inherited from the reference sketch.
5. Check motion readings, GPS reception, and modem status. A GPS fix may need an outdoor view of the sky.

The dashboard is local and does not require internet access. Opening the external map does require internet. Anyone connected to the open access point can access the controls and settings.

## Alarm and SMS behavior

- Activation turns on the siren and toggles the strobe every 500 ms.
- SOS is queued immediately. Subsequent blasts are due 60 seconds after the previous start; blasts do not overlap, so modem delays can extend this interval.
- Each blast takes one snapshot of the name, recipients, and sensor values. Recipients are sent messages one at a time, not simultaneously.
- Cancelling stops the relays and remaining recipients. An SMS already submitted to the modem cannot be recalled.
- Returning inside the rotation threshold does not cancel an active alarm.

Example format, with illustrative readings:

```text
AQUAWATCH DISTRESS MESSAGE
Device: Spike
lat; long: 14.332070,120.957080
Gyro X: 50.0 deg Y: -12.0 deg
Acceleration: 0.20 m/s2
```

If the GPS fix is missing or at least 10 seconds old, the SMS reports **No fresh fix**. Modem acceptance is not a recipient delivery receipt.

## Configuration

| Setting | Default / behavior |
| --- | --- |
| Wi-Fi name | `aquawatch_dev`, open access point |
| Dashboard | `http://192.168.4.1` |
| Device name | `Spike`; editable and stored separately from contacts in EEPROM |
| Name length | 1–16 letters, numbers, spaces, dots, underscores, or hyphens |
| Contacts | Up to six, stored in EEPROM; current removal logic retains at least one |
| Button holds | 1.5 seconds to start, 3 seconds to stop; one action per press |
| Automatic threshold | ±50° on either displayed rotation axis |
| Automatic re-arm | Both displayed axes strictly within ±45° |
| Signal polling | About every 10 seconds while the modem is free; SMS takes priority |
| Debug serial | 115200 baud |

## Development status

| Component | Current boundaries |
| --- | --- |
| Rotation | Integrates gyro rates relative to startup, using the axis mapping and deadband in `gyro.ino`. This is not absolute gravity-referenced tilt and can drift. |
| Acceleration | Uses the reference sketch's gravity-removal estimate, not full orientation-aware sensor fusion. |
| Cellular | Requires a compatible network, working antenna, SIM service, and stable modem power. Signal can display unknown or stale. |
| Web telemetry | Detects malformed responses, timeouts, and connection errors and retries; actual device-side issues still need hardware diagnostics. |
| Integration | This sketch provides local Wi-Fi and SMS operation. It does not synchronize with the separate AquaWatch mobile app or backend. |
| Validation | NodeMCU compilation and mocked alarm/dashboard checks passed; this does not verify physical relay timing or SMS delivery. |

## Project structure

```text
aquawatch_micro/
├── aquawatch_build/
│   ├── aquawatch_build.ino   # Open this in Arduino IDE
│   └── aquawatch_logo.h     # Embedded transparent PNG
├── aquawatch.ino            # Editable source; synchronized with upload sketch
├── aquawatch_logo.h         # Matching logo header
├── aquawatch-logo.png       # Transparent logo used by this README
├── aquawatch-logo.jpg       # Earlier logo export; not used by current firmware
├── gyro.ino                 # Motion calculation reference
├── inspo.ino                # Earlier ESP32/dashboard reference
├── check_features.cjs       # Mocked alarm and dashboard checks
├── FEATURES.md              # Implementation and troubleshooting notes
└── README.md
```

## Development

Keep the root source and upload copies synchronized when editing. In PowerShell:

```powershell
Copy-Item aquawatch.ino aquawatch_build/aquawatch_build.ino -Force
Copy-Item aquawatch_logo.h aquawatch_build/aquawatch_logo.h -Force
node check_features.cjs
arduino-cli compile --fqbn esp8266:esp8266:nodemcuv2 aquawatch_build
```

The host checks require Node.js and mock hardware/browser inputs. They cover rotation thresholds, re-arming, button holds, dashboard rendering, malformed responses, timeouts, and recovery. DFRobot_BMI160 1.0.0 may emit `LITTLE_ENDIAN` redefinition and `Wire.requestFrom` overload warnings with this core.

If the dashboard freezes, check the visible connection error and **http://192.168.4.1/api** while connected to the device. See [FEATURES.md](FEATURES.md) for additional notes.
