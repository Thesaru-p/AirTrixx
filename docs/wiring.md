# Wiring

## Wristband ESP32-C3

MPU6050 I2C:

| Sensor | ESP32-C3 |
|---|---|
| MPU6050 SDA | GPIO8 |
| MPU6050 SCL | GPIO9 |
| MPU6050 VCC | 3V3 |
| MPU6050 GND | GND |

Firmware constants:

```cpp
#define SDA_PIN 8
#define SCL_PIN 9
#define MPU6050_ADDR 0x68
```

## Cam Dock ESP32-S3

### I2C Buses

Servo PCA9685 bus:

| Signal | ESP32-S3 |
|---|---|
| SDA | GPIO18 |
| SCL | GPIO21 |

ToF mux/sensor bus:

| Signal | ESP32-S3 |
|---|---|
| SDA | GPIO15 |
| SCL | GPIO17 |

### I2C Addresses

| Device | Address |
|---|---:|
| PCA9685 | `0x40` |
| TCA9548A mux | `0x70` |

### TCA9548A Mux Channels

| Sensor | Mux channel |
|---|---:|
| Left VL53L1X ToF | 2 |
| Right VL53L1X ToF | 1 |

### PCA9685 Servo Channels

| Servo | PCA9685 channel |
|---|---:|
| R_PAN | 10 |
| R_TILT | 11 |
| CAM_PAN | 12 |
| CAM_TILT | 13 |
| L_PAN | 14 |
| L_TILT | 15 |

### Power Notes

- Power servos from a separate regulated servo supply sized for peak current.
- Tie servo supply ground, PCA9685 ground, and ESP32 ground together.
- Power VL53L1X sensors at the voltage supported by their breakout boards.
- Keep I2C wires short or lower bus speed if reads are unreliable.

## Fan Controller ESP32-C3

Firmware target: `firmware/fan_controller_esp32c3`.

| Signal | ESP32-C3 |
|---|---|
| 2N2222A base via 470 ohm resistor | GPIO6 |
| DS18B20 data, both sensors on one OneWire bus | GPIO0 |

The fan output is active-high because the schematic uses an NPN low-side switch.
