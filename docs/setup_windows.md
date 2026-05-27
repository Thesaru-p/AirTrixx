# Windows Setup

## Arduino IDE

Install Arduino IDE 2.x and add the ESP32 board package:

1. Open Arduino IDE.
2. Go to **File > Preferences**.
3. Add this Boards Manager URL:

```text
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

4. Open **Tools > Board > Boards Manager**.
5. Install **esp32 by Espressif Systems**.

## Board Settings

### Antenna ESP32-S3

- Board: an ESP32-S3 dev board matching your hardware
- USB CDC On Boot: Enabled if available
- Upload Mode: UART0 / Hardware CDC as appropriate for the board
- CPU Frequency: 240 MHz
- Flash Frequency: 80 MHz
- Partition Scheme: Default

### Cam Dock ESP32-S3

Use the same ESP32-S3 family settings as the Antenna. Confirm that GPIOs 15, 17, 18, and 21 are available on your board and not reserved by PSRAM/flash.

### Wristband ESP32-C3

- Board: an ESP32-C3 dev board matching your hardware
- USB CDC On Boot: Enabled if available
- CPU Frequency: 160 MHz
- Partition Scheme: Default

## Required Arduino Libraries

Install through **Sketch > Include Library > Manage Libraries**:

- ESP32 Arduino core by Espressif Systems
- Adafruit PWM Servo Driver Library
- VL53L1X by Pololu

The Wristband firmware talks directly to the MPU6050 over `Wire`, so it does not need a separate IMU Arduino library.

The Cam Dock code isolates VL53L1X-specific calls in:

- `initToFSensor(...)`
- `readToFLeft()`
- `readToFRight()`

If your ToF library uses different method names, adapt those helpers only.

## Shared Firmware Headers

The sketches include:

```cpp
#include "../shared/AirTrixxConfig.h"
#include "../shared/AirTrixxProtocol.h"
```

If your Arduino IDE setup does not allow parent-folder relative includes, copy these two shared headers into the sketch folder being compiled. Keep the canonical copies in `firmware/shared` and copy again after editing MAC addresses.

## Python Setup

From the repo root:

```powershell
cd python_app
py -m venv .venv
.\.venv\Scripts\activate
pip install --upgrade pip
pip install -r requirements.txt
python main.py
```

If MediaPipe throws an error about `solutions` or `MessageFactory`, reinstall the pinned GUI dependencies inside the virtual environment:

```powershell
pip install --force-reinstall -r requirements.txt
```

Default serial baud is `921600`. If the USB serial link is unstable, change both:

- `AIRTRIXX_SERIAL_BAUD` in `firmware/shared/AirTrixxConfig.h`
- `serial_baud` in `python_app/config.py`

to `115200`, then reflash the Antenna.

## COM Port Troubleshooting

- Flash `mac_finder.ino` first to confirm each board enumerates.
- Close Arduino Serial Monitor before connecting from the Python GUI.
- In Device Manager, look under **Ports (COM & LPT)**.
- If the Antenna JSON view remains empty, confirm:
  - the Antenna firmware is flashed
  - the Python baud rate matches firmware
  - `DEBUG_SERIAL` is `false` for clean JSON
  - the GUI is connected to the Antenna COM port, not the Cam Dock or Wristband
