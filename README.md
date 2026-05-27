# AirTrixx Prototype

AirTrixx is a multi-sensor laptop control prototype that combines:

- a USB-connected Antenna ESP32-S3 host bridge
- ESP-NOW wireless devices
- a Cam Dock ESP32-S3 with two VL53L1X ToF sensors and PCA9685 servo control
- a Wristband ESP32-C3 with MPU6050 IMU sensing and periodic battery telemetry
- a Fan Controller ESP32-C3 with DS18B20 temperature telemetry and GUI on/off control
- a Keyboard ESP32-S3 with three VL53L0X ToF lanes through a TCA9548A mux
- a Windows Python app using OpenCV, MediaPipe Hands, pyserial, NumPy, Pillow, and Tkinter

The current firmware targets are:

- `firmware/antenna_esp32s3/antenna_esp32s3.ino`
- `firmware/camdock_esp32s3/camdock_esp32s3.ino`
- `firmware/wristband_esp32c3/wristband_esp32c3.ino`
- `firmware/fan_controller_esp32c3/fan_controller_esp32c3.ino`
- `firmware/keyboard_esp32s3/keyboard_esp32s3.ino`
- `firmware/mac_finder/mac_finder.ino`

Future devices are represented in the Antenna JSON model as placeholders only:
charging dock and audio dock.

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
  gui.py
  status_app.py
  requirements.txt
  config/calibration.json  # auto-created locally and ignored by git
  data/gestures/
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

If your Arduino IDE cannot resolve the shared headers from `firmware/shared`, copy `AirTrixxConfig.h` and `AirTrixxProtocol.h` into the sketch folder you are compiling, or compile with Arduino CLI from the repo so the relative include paths are preserved.

## Run The Windows Python App

From the repo root:

```powershell
cd AirTrixx
py -m venv python_app\.venv
.\python_app\.venv\Scripts\activate
pip install -r python_app\requirements.txt
python python_app\main.py
```

Or from inside `python_app`:

```powershell
cd python_app
py -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

The GUI provides:

- COM port selection and connect/disconnect
- live MediaPipe camera preview
- live Antenna JSON/device state display
- live fused input array display
- fan on/off control, live temperature readings, and wireless fan firmware flash
- keyboard ToF live data and a 0-300 mm distance grid
- camera center command
- servo calibration save/load
- labeled gesture recording to `python_app/data/gestures/<gesture_name>/`

To run the smaller connection status checker without camera or MediaPipe:

```powershell
python status_app.py
```

It shows the USB Antenna link, JSON stream freshness, and the latest device status for Cam Dock, Wristband, Fan Controller, and future placeholder devices.

## Current Limitations

- Wristband battery telemetry reports voltage and percentage to the Antenna about every 5 minutes.
- Wristband yaw is not currently reported; the sealed wristband firmware uses the MPU6050 only.
- VL53L1X code targets the Pololu `VL53L1X` Arduino library. If you use another library, adapt only the isolated ToF init/read helpers in `camdock_esp32s3.ino`.
- Servo center values are defaults and should be calibrated per build in `python_app/config/calibration.json`.
