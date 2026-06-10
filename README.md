# AirTrixx Prototype

AirTrixx is a multi-sensor laptop control prototype that combines:

- a USB-connected Antenna ESP32-S3 host bridge
- ESP-NOW wireless devices
- a Cam Dock ESP32-S3 with two VL53L1X ToF sensors and PCA9685 servo control
- a Wristband ESP32-C3 with MPU6050 IMU sensing and periodic battery telemetry
- a Fan Controller ESP32-C3 with DS18B20 temperature telemetry and GUI on/off control
- a Keyboard ESP32-S3 with VL53L0X ToF lanes through a TCA9548A mux, Antenna ESP-NOW word training, and live typed-word prediction
- a Charging Dock ESP32-S3 with four INA219 charging channels and ESP-NOW status telemetry
- a packaged Python desktop app using Tkinter, OpenCV, MediaPipe Tasks, pyserial, NumPy, Pillow, and pynput

The current firmware targets are:

- `firmware/antenna_esp32s3/antenna_esp32s3.ino`
- `firmware/camdock_esp32s3/camdock_esp32s3.ino`
- `firmware/wristband_esp32c3/wristband_esp32c3.ino`
- `firmware/fan_controller_esp32c3/fan_controller_esp32c3.ino`
- `firmware/keyboard_esp32s3/keyboard_esp32s3.ino`
- `firmware/charging_dock_esp32s3/src/main.cpp`
- `firmware/mac_finder/mac_finder.ino`

Future devices are represented in the Antenna JSON model as placeholders only
until their firmware target is active.

## Repo Layout

```text
firmware/
  shared/
    AirTrixxConfig.h
    AirTrixxProtocol.h
  mac_finder/
  antenna_esp32s3/
  camdock_esp32s3/
  wristband_esp32c3/
  fan_controller_esp32c3/
  keyboard_esp32s3/
python_app/
  main.py
  config.py
  serial_bridge.py
  mediapipe_tracker.py
  servo_controller.py
  gesture_recorder.py
  fusion_state.py
  input_mapper.py
  input_backend.py
  keyboard_bridge.py
  keyboard_model.py
  gui.py
  status_app.py
  requirements.txt
packaging/
  build_macos_dmg.sh
  build_windows_installer.ps1
  AirTrixx.spec
docs/
  setup_windows.md
  protocol.md
  wiring.md
```

## Find MAC Addresses

1. Open `firmware/mac_finder/mac_finder.ino` in Arduino IDE.
2. Select the correct board, port, and flash it to each ESP32 board.
3. Open Serial Monitor at `115200`.
4. Copy the printed array:

```cpp
uint8_t DEVICE_MAC[] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};
```

5. Paste each discovered MAC into `firmware/shared/AirTrixxConfig.h`:

```cpp
static uint8_t ANTENNA_MAC_PLACEHOLDER[6] = {...};
static uint8_t CAMDOCK_MAC_PLACEHOLDER[6] = {...};
static uint8_t WRISTBAND_MAC_PLACEHOLDER[6] = {...};
static uint8_t FANS_MAC_PLACEHOLDER[6] = {...};
static uint8_t KEYBOARD_MAC_PLACEHOLDER[6] = {...};
static uint8_t CHARGING_DOCK_MAC_PLACEHOLDER[6] = {...};
```

Keep `ESPNOW_CHANNEL = 1` on every target.

## Flash Firmware

Install the Arduino libraries listed in [docs/setup_windows.md](docs/setup_windows.md).

Flash in this order:

1. `firmware/mac_finder/mac_finder.ino` on every board to collect MACs.
2. Edit `firmware/shared/AirTrixxConfig.h` with the discovered MAC addresses.
3. Flash `firmware/antenna_esp32s3/antenna_esp32s3.ino` to the USB Antenna ESP32-S3.
4. Flash `firmware/camdock_esp32s3/camdock_esp32s3.ino` to the Cam Dock ESP32-S3.
5. Flash `firmware/wristband_esp32c3/wristband_esp32c3.ino` to the Wristband ESP32-C3.
6. Flash `firmware/fan_controller_esp32c3/fan_controller_esp32c3.ino` to the Fan Controller ESP32-C3.
7. Flash `firmware/keyboard_esp32s3/keyboard_esp32s3.ino` to the Keyboard ESP32-S3.
8. Flash `firmware/charging_dock_esp32s3` to the Charging Dock ESP32-S3 with PlatformIO.

If your Arduino IDE cannot resolve the shared headers from `firmware/shared`, copy `AirTrixxConfig.h` and `AirTrixxProtocol.h` into the sketch folder you are compiling, or compile with Arduino CLI from the repo so the relative include paths are preserved.

## Run The Python App From Source

From the repo root:

```powershell
cd AirTrixx
py -m venv python_app\.venv
.\python_app\.venv\Scripts\activate
pip install -r python_app\requirements.txt
python packaging\download_models.py
python python_app\main.py
```

On macOS/Linux, use the same steps with `python3` and `source`:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r python_app/requirements.txt
python packaging/download_models.py
python python_app/main.py
```

The GUI provides:

- COM port selection and connect/disconnect
- live MediaPipe camera preview
- live Antenna JSON/device state display
- live fused input array display
- per-signal keyboard and mouse mappings with tap, hold, repeat, scroll, and movement actions
- fan on/off control, live temperature readings, and wireless fan firmware flash
- keyboard ToF live data, a 0-300 mm distance grid, word training controls, prediction log, and default detected-word typing
- charging dock connection and per-channel charging status
- camera center command
- servo calibration save/load
- labeled gesture recording under `python_app/data/gestures`

Runtime config, mappings, logs, and temporary audio files are stored outside the app bundle:

- Windows: `%APPDATA%\AirTrixx`
- macOS: `~/Library/Application Support/AirTrixx`

Keyboard training data and the locally retrained model are stored under the same user data root in `keyboard`. The app installs the bundled starter model on first launch, then replaces it when you train new words from the Keyboard page.

## Package The App

macOS Apple Silicon DMG:

```bash
./packaging/build_macos_dmg.sh
```

Windows x64 installer or portable zip:

```powershell
.\packaging\build_windows_installer.ps1
```

The build scripts install dependencies into an isolated build venv, generate icons, download the MediaPipe hand-landmarker model into generated packaging assets, and run PyInstaller. Windows builds must be produced on Windows; macOS DMGs must be produced on macOS.

On Windows, the installer registers `AirTrixx.exe` with the high-performance GPU preference under the current user's graphics settings. Portable zip builds self-register this preference on first launch and need one restart before Windows applies it. OpenCV OpenCL acceleration is enabled at startup when the installed OpenCV build and GPU driver support it. MediaPipe 0.10.9's Python GPU delegate is not supported on Windows, so hand landmark inference still falls back to CPU on Windows builds.

To run the smaller connection status checker without camera or MediaPipe:

```powershell
python status_app.py
```

It shows the USB Antenna link, JSON stream freshness, and the latest device status for Cam Dock, Wristband, Fan Controller, and future placeholder devices.

## Current Limitations

- Wristband battery telemetry reports voltage and percentage to the Antenna about every 5 minutes.
- Wristband yaw is not currently reported; the sealed wristband firmware uses the MPU6050 only.
- VL53L1X code targets the Pololu `VL53L1X` Arduino library. If you use another library, adapt only the isolated ToF init/read helpers in `camdock_esp32s3.ino`.
- Servo center values are defaults and should be calibrated per build from the app Settings page.
- Unsigned packaged builds may show Windows SmartScreen or macOS Gatekeeper warnings until code-signing/notarization is added.
