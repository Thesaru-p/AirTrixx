# AirTrixx Protocol

AirTrixx uses two protocols:

- ESP-NOW binary structs between ESP32 boards
- newline-delimited JSON over USB serial between the Antenna and Windows Python app

ESP-NOW stays binary for low latency. Only the Antenna prints JSON.

## Constants

`ESPNOW_CHANNEL = 1`

Device IDs:

| ID | Device |
|---:|---|
| 1 | antenna |
| 2 | camdock |
| 3 | wristband |
| 4 | keyboard |
| 5 | charging_dock |
| 6 | audiodock |
| 7 | fans |

Message types:

| Name | Value |
|---|---:|
| `MSG_WRISTBAND_DATA` | 1 |
| `MSG_CAMDOCK_DATA` | 2 |
| `MSG_SERVO_COMMAND` | 3 |
| `MSG_HEARTBEAT` | 4 |
| `MSG_ACK` | 5 |
| `MSG_BATTERY_STATUS` | 6 |
| `MSG_OTA_START` | 7 |
| `MSG_FAN_STATUS` | 8 |
| `MSG_FAN_COMMAND` | 9 |

Active servo pairs:

| Name | Value |
|---|---:|
| none | 0 |
| right | 1 |
| left | 2 |
| camera | 3 |
| hands | 4 |

## ESP-NOW Header

All packets begin with:

```cpp
struct AirTrixxPacketHeader {
  uint8_t protocol_version;
  uint8_t msg_type;
  uint8_t device_id;
  uint16_t sequence;
  uint32_t t_ms;
  uint8_t battery_low;
};
```

All structs are packed.
`battery_low` is a legacy header field. Battery level is sent separately with
`MSG_BATTERY_STATUS`.

## Wristband Data

```cpp
struct WristbandDataPacket {
  AirTrixxPacketHeader header;
  int16_t accel_mg_x;
  int16_t accel_mg_y;
  int16_t accel_mg_z;
  int32_t gyro_mdps_x;
  int32_t gyro_mdps_y;
  int32_t gyro_mdps_z;
  int16_t pitch_cdeg;
  int16_t roll_cdeg;
};
```

Scaling:

- `accel_mg_*`: milligravity. Convert to m/s^2 with `value * 0.00980665`.
- `gyro_mdps_*`: millidegrees/second. Convert to deg/s with `value * 0.001`.
- `pitch_cdeg`, `roll_cdeg`: centidegrees. Convert to degrees with `value * 0.01`.

## Battery Status

Wristband battery telemetry is sent 5 seconds after the wristband confirms the
Antenna ESP-NOW link, then about every 5 minutes while connected.

```cpp
struct BatteryStatusPacket {
  AirTrixxPacketHeader header;
  uint16_t battery_mv;
  uint8_t battery_percent;
  uint8_t battery_valid;
  uint16_t battery_adc_raw;
};
```

## OTA Start

The Antenna forwards this packet to the selected wireless target when the GUI
starts a wireless flash. The target stops normal ESP-NOW, joins the provided
Wi-Fi network, downloads `url`, applies the HTTP OTA image, then reboots into
normal ESP-NOW mode. Currently supported targets are `wristband` and `fans`.

```cpp
struct OtaStartPacket {
  AirTrixxPacketHeader header;
  char ssid[33];
  char password[65];
  char url[97];
  char md5[33];
};
```

## Cam Dock Data

```cpp
struct CamDockDataPacket {
  AirTrixxPacketHeader header;
  uint16_t left_tof_mm;
  uint16_t right_tof_mm;
  uint8_t active_target;
};
```

`active_target` uses the same values as active servo pairs:
none, right, left, camera, hands.

## Fan Controller

The fan controller sends this status packet at about 2 Hz:

```cpp
struct FanStatusPacket {
  AirTrixxPacketHeader header;
  uint8_t fan_on;
  uint8_t temp1_valid;
  uint8_t temp2_valid;
  int16_t temp1_centi_c;
  int16_t temp2_centi_c;
  uint16_t last_command_sequence;
};
```

Temperatures are centi-degrees Celsius. The Antenna forwards GUI on/off
requests as:

```cpp
struct FanCommandPacket {
  AirTrixxPacketHeader header;
  uint8_t fan_on;
};
```

## Servo Commands

PC JSON commands are parsed by the Antenna and converted to:

```cpp
struct ServoCommandPacket {
  AirTrixxPacketHeader header;
  uint8_t active_pair;
  uint8_t active_mask;
  uint8_t disable_unused;
  uint16_t r_pan;
  uint16_t r_tilt;
  uint16_t l_pan;
  uint16_t l_tilt;
  uint16_t cam_pan;
  uint16_t cam_tilt;
};
```

Servo pulses are direct PCA9685 ticks from `0` to `4095`.

- `0` means disable PWM output for that servo channel.
- Nonzero values are passed to `setPWM(channel, 0, pulse)`.
- `active_mask` tells the Cam Dock which two servo channels should be updated.
- `disable_unused = true` disables every servo channel outside `active_pair`.

Servo mask bits:

| Bit | Servo |
|---:|---|
| 0 | right pan |
| 1 | right tilt |
| 2 | left pan |
| 3 | left tilt |
| 4 | camera pan |
| 5 | camera tilt |

## Antenna To PC JSON

The Antenna prints one JSON object per line at about 30 Hz.

Example:

```json
{
  "t_ms": 123456,
  "sequence": 42,
  "devices": {
    "wristband": {
      "status": "ok",
      "battery_level": 87,
      "battery_voltage": 4.087,
      "battery": {
        "status": "ok",
        "percent": 87,
        "voltage_v": 4.087,
        "adc_raw": 2846,
        "age_ms": 2200
      },
      "sequence": 10,
      "t_ms": 123430,
      "accel": {"x": 0.01, "y": 0.02, "z": 9.81},
      "gyro": {"x": 0.1, "y": 0.2, "z": 0.3},
      "pitch": 1.2,
      "roll": -2.5,
      "yaw": null
    },
    "camdock": {
      "status": "ok",
      "battery_level": null,
      "sequence": 8,
      "t_ms": 123440,
      "tof": {"left_mm": 520, "right_mm": 610},
      "active_target": "right"
    },
    "keyboard": {"status": "TBD", "input": "TBD", "battery_level": null},
    "charging_dock": {"status": "TBD", "input": "TBD", "battery_level": null},
    "audiodock": {"status": "TBD", "input": "TBD", "battery_level": null},
    "fans": {
      "status": "ok",
      "input": "on",
      "battery_level": null,
      "fan_on": true,
      "sequence": 12,
      "t_ms": 123450,
      "last_command_sequence": 3,
      "temps": {"sensor_1_c": 31.25, "sensor_2_c": 32.00}
    }
  }
}
```

Disconnected implemented devices report `status: "not_connected"`.
Future placeholder devices report `status: "TBD"`.

## PC To Antenna JSON

The Python app sends newline-delimited JSON commands:

```json
{
  "cmd": "servo",
  "target": "camdock",
  "active_pair": "right",
  "disable_unused": true,
  "servos": {
    "r_pan": 320,
    "r_tilt": 360,
    "l_pan": 0,
    "l_tilt": 0,
    "cam_pan": 0,
    "cam_tilt": 0
  }
}
```

Current behavior:

- Both hands visible: active pair `hands`; both ToF pan/tilt pairs update; camera outputs disabled.
- Only right hand visible: active pair `right`; right ToF pan/tilt update; left/camera outputs disabled.
- Only left hand visible: active pair `left`; left ToF pan/tilt update; right/camera outputs disabled.
- No hand visible: active pair `none`; all hand-tracking servos disabled.
- Camera calibration command: active pair `camera`; camera pan/tilt receive center pulses.

Fan control commands are:

```json
{
  "cmd": "fans",
  "target": "fans",
  "fan_on": true
}
```

Fan controller OTA commands use the same `cmd: "ota"` shape as wristband OTA,
with `"target": "fans"`.
