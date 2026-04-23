# Bare-Metal Sensor Drivers for ESP32

> Portfolio project demonstrating bare-metal I2C and 1-Wire driver development on ESP32 — no HAL, no libraries, register-level implementation for MAX30102, BMP280, and DS18B20.

---

## Overview

This module provides production-quality, HAL-free sensor drivers written from scratch for the ESP32 platform. The drivers are designed for use in a secure real-time patient monitoring system and are integrated into a multi-task FreeRTOS firmware architecture.

The project demonstrates:
- **Bare-metal protocol implementation** — direct register-level I2C and 1-Wire communication, no abstraction libraries
- **Shared bus architecture** — a reusable `i2c_bus` abstraction layer manages multiple devices on the same bus
- **Security-aware design** — sensor data feeds directly into an mbedTLS-secured pipeline, with threat modelling applied at the firmware level (STRIDE, IEC 62443)

---

## Hardware Components

| Sensor | Protocol | Measurement | Notes |
|--------|----------|-------------|-------|
| MAX30102 | I2C | SpO2 / Heart Rate | Interrupt-driven data ready handling |
| BMP280 | I2C | Temperature / Pressure | Shared I2C bus with MAX30102 |
| DS18B20 | 1-Wire | Temperature | Parasite power mode supported |

All three sensors are managed through custom drivers — no ESP-IDF sensor component or third-party libraries are used.

---

## Project Structure

```
├── include/
│   ├── drivers/
│   │   ├── i2c_bus.h           # Shared I2C bus abstraction
│   │   ├── max30102.h          # SpO2/HR sensor driver
│   │   ├── bmp280.h            # Temperature/Pressure sensor driver
│   │   └── ds18b20_sensor.h    # 1-Wire temperature sensor driver
│   ├── tasks/
│   │   ├── pulse_oximeter_task.h
│   │   └── temperature_monitor_task.h
│   └── app/
│       └── project_config.h
├── src/
│   ├── drivers/
│   │   ├── i2c_bus.c           # I2C bus init, read/write primitives
│   │   ├── max30102.c          # Register map, FIFO management, SpO2 calc
│   │   ├── bmp280.c            # Compensation formulas, config registers
│   │   └── ds18b20_sensor.c    # 1-Wire timing, ROM commands, scratchpad
│   ├── tasks/
│   │   ├── pulse_oximeter_task.c
│   │   └── temperature_monitor_task.c
│   └── main.c
├── test/
├── CMakeLists.txt
└── README.md
```

---

## Driver Architecture

### I2C Bus Abstraction (`i2c_bus`)

Rather than initialising the I2C peripheral independently in each driver, a shared `i2c_bus` layer handles:
- ESP32 I2C master initialisation and configuration
- Generic read/write primitives used by all I2C devices
- Bus error handling and retry logic

This design allows MAX30102 and BMP280 to share the same I2C bus cleanly, with no duplicated peripheral setup.

### MAX30102 — SpO2 & Heart Rate

- Full register map implementation (mode config, SpO2 config, FIFO control)
- Interrupt-driven FIFO data-ready handling
- Red/IR LED current and sample rate configurable via `project_config.h`
- SpO2 and heart rate calculated from raw FIFO samples

### BMP280 — Temperature & Pressure

- Reads factory calibration coefficients from NVM on init
- Applies official Bosch compensation formulas for temperature and pressure
- Configurable oversampling, standby time, and filter coefficient

### DS18B20 — 1-Wire Temperature

- Full 1-Wire bus implementation: reset pulse, presence detect, bit-level read/write
- Supports ROM commands: `SKIP ROM`, `MATCH ROM`, `SEARCH ROM`
- Reads scratchpad register and applies 12-bit resolution conversion
- Parasite power mode supported

---

## FreeRTOS Task Architecture

Sensor drivers are consumed by two independent FreeRTOS tasks:

| Task | Sensors Used | Stack | Priority |
|------|-------------|-------|----------|
| `pulse_oximeter_task` | MAX30102 | Configurable | High |
| `temperature_monitor_task` | BMP280, DS18B20 | Configurable | Medium |

Tasks perform physiological data validation before forwarding readings downstream. Invalid readings (out-of-range SpO2, temperature spikes) are discarded and logged.

---


## Build & Flash

### Prerequisites

- ESP-IDF v5.x
- CMake ≥ 3.16
- Python 3.8+

### Build

```bash
git clone https://github.com/tomehabb/Bare-Metal-Sensor-Drivers-for-ESP32-Secure-IoT-Patient-Monitoring.git
cd Bare-Metal-Sensor-Drivers-for-ESP32-Secure-IoT-Patient-Monitoring
idf.py set-target esp32
idf.py menuconfig   # Set Wi-Fi credentials and MQTT broker in project config
idf.py build
```

### Flash

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Skills Demonstrated

`ESP32` · `Bare-metal Firmware` · `I2C` · `1-Wire` · `FreeRTOS` · `IEC 62443` · `C` · `CMake`

---

## Author

**Thomas Ibrahim**
