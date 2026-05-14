# 3-Axis Gimbal Stabiliser — Arduino Nano / ATmega328P

A bare-metal C firmware for a 3-axis gimbal stabiliser built on an **Arduino Nano (ATmega328P)** using a **Pololu AltIMU-10 v6** sensor board and three **SG90 servo motors**. No Arduino libraries — direct register manipulation only.

> Built on top of the [Embedded Systems course repository](https://github.com/mamuleanu/embedded-systems-course-atmega328p) for the Automation and Applied Informatics programme, Faculty of Automation, Computers and Electronics, University of Craiova.

[![Run Tests](https://github.com/mamuleanu/embedded-systems-course-atmega328p/actions/workflows/tests.yml/badge.svg)](https://github.com/mamuleanu/embedded-systems-course-atmega328p/actions/workflows/tests.yml)

---

## What it does

At power-on the firmware reads the physical orientation of the sensor and captures it as the **home position**. From that point on, three independent PID controllers work continuously to hold that orientation — compensating for any tilt or rotation applied to the handle by driving the three servos.

**Control loop (runs at ~208 Hz):**
1. Read accel + gyro from LSM6DSO
2. Read heading from LIS3MDL magnetometer
3. Feed both into the Madgwick AHRS filter → clean pitch, roll, yaw angles
4. Each PID computes how much to move its servo to cancel the measured error
5. Drive the three servos to the corrected positions

---

## Hardware

| Component | Part |
|-----------|------|
| Microcontroller | Arduino Nano (ATmega328P, 16 MHz) |
| IMU sensor board | Pololu AltIMU-10 v6 (LSM6DSO + LIS3MDL + LPS22DF) |
| Servos | 3× SG90 |

### Wiring

| Signal | Arduino Nano pin |
|--------|-----------------|
| I2C SDA | A4 |
| I2C SCL | A5 |
| Servo X (pitch) | D9 |
| Servo Y (roll) | D10 |
| Servo Z (yaw) | D3 |

**Power:** servos must be powered directly from the supply (5 V / ≥ 2 A), **not** through the Arduino 5 V pin. Share a common GND between the supply, Arduino, and all servos. Add a 100 µF capacitor across the servo power rail to absorb current spikes.

---

## Project Structure

```
├── bsp/
│   ├── bsp.h               # Common BSP include
│   ├── nano.h              # Arduino Nano pin definitions
│   └── uno.h               # Arduino Uno pin definitions
│
├── drivers/
│   ├── adc/
│   │   ├── adc.c/h         # 10-bit ADC, blocking conversion
│   ├── eeprom/
│   │   ├── eeprom.c/h      # EEPROM read, write, update (lifespan-aware)
│   ├── gpio/
│   │   ├── gpio.c/h        # GPIO init, read, write, toggle
│   ├── i2c/
│   │   ├── i2c.c/h         # I2C master, 400 kHz, timeout protection,
│   │   │                   # I2C_Scan(), I2C_IsDevicePresent()
│   ├── imu/
│   │   ├── lsm6dso.c/h     # ✅ ACTIVE — accel + gyro, 208 Hz (AltIMU-10 v6)
│   │   ├── lis3mdl.c/h     # ✅ ACTIVE — magnetometer, 80 Hz (AltIMU-10 v6)
│   │   ├── lps22df.c/h     # 🔧 AVAILABLE — barometer / altimeter (AltIMU-10 v6)
│   │   └── lsm6ds33.c/h   # 🔧 AVAILABLE — accel + gyro (AltIMU-10 v5, legacy)
│   ├── interrupt/
│   │   ├── external_interrupt.c/h  # 🔧 AVAILABLE — INT0/INT1 with callbacks
│   ├── pwm/
│   │   ├── pwm.c/h         # 🔧 AVAILABLE — high-level PWM wrapper (Timer1/Timer2)
│   ├── servo/
│   │   ├── servo.c/h       # ✅ ACTIVE — SG90 driver, safe limits 10°–170°
│   ├── timer/
│   │   ├── timer0.c/h      # ✅ ACTIVE — 1 ms Millis() system tick (CTC mode)
│   │   ├── timer1.c/h      # ✅ ACTIVE — 16-bit 50 Hz PWM (servo X/Y)
│   │   └── timer2.c/h      # ✅ ACTIVE — 8-bit ~61 Hz PWM (servo Z)
│   └── usart/
│       ├── usart.c/h       # 🔧 AVAILABLE — UART driver (future serial telemetry)
│
├── src/
│   ├── main.c              # Entry point, fault LED on sensor failure
│   ├── gimbal.c            # Full control loop: sensors → Madgwick → PID → servos
│   └── gimbal.h            # All tuning parameters (Kp, Ki, Kd, beta, limits)
│
├── utils/
│   ├── madgwick.c/h        # ✅ ACTIVE — Madgwick AHRS (9-DOF with mag, 6-DOF fallback)
│   ├── pid.c/h             # ✅ ACTIVE — PID controller with anti-windup
│   ├── filter.c/h          # 🔧 AVAILABLE — complementary filter (simpler alternative)
│   ├── delay.c/h           # Busy-wait delay helpers
│   └── utils.h             # Bit-manipulation macros (SET_BIT, CLEAR_BIT, etc.)
│
├── test/
│   ├── framework/
│   │   └── test_framework.h        # Minimal host-side test runner
│   ├── mocks/
│   │   ├── avr/                    # Mock avr/io.h and avr/interrupt.h
│   │   └── registers.c             # Simulated ATmega328P registers
│   ├── test_gpio.c                 # GPIO unit tests
│   ├── test_pwm.c                  # Timer PWM unit tests
│   └── test_pwm_wrapper.c          # PWM wrapper unit tests
│
└── Makefile
```

> **✅ ACTIVE** — compiled and used by the gimbal application.  
> **🔧 AVAILABLE** — implemented and ready to use, not included in the current build.

---

## Features

### Custom gimbal application
- **Madgwick AHRS filter** — fuses accel, gyro, and magnetometer into drift-free pitch, roll, and yaw. Falls back to 6-DOF (accel + gyro only) when no new magnetometer data is available
- **Home position capture** — at startup the firmware runs the filter for 1 second to converge, then captures the current orientation as the hold target. Whatever position the rig is in at power-on becomes the new "level"
- **3 independent PID controllers** — one per axis with configurable Kp, Ki, Kd and output clamping. Yaw uses `Ki = 0` to prevent integral drift (yaw has no gravity reference, only the magnetometer)
- **Safe servo limits** — all axes are hardware-clamped to 10°–170° to protect the SG90 motors from stalling at their mechanical endstops
- **Servo axis mapping** — D9 (Timer1 OC1A) for X/pitch, D10 (Timer1 OC1B) for Y/roll, D3 (Timer2 OC2B) for Z/yaw

### Drivers (bare-metal, no Arduino libraries)
- **I2C master** — blocking, 400 kHz, with timeout protection on every bus wait, `I2C_IsDevicePresent()`, and `I2C_Scan()`
- **LSM6DSO** — accel + gyro at 208 Hz, direct STATUS\_REG polling
- **LIS3MDL** — magnetometer at 80 Hz, ±4 gauss, ultra-high-performance mode
- **GPIO** — init, read, write, toggle
- **Timers** — Timer0 (1 ms `Millis()` tick), Timer1 (16-bit 50 Hz PWM), Timer2 (8-bit ~61 Hz PWM)
- **EEPROM** — read, write, update (lifespan-aware)
- **ADC** — blocking 10-bit conversion

---

## Build & Flash

### Prerequisites
- `avr-gcc` toolchain
- `avrdude` (v7.3 recommended — v8.x has a known macOS/CH340 reset bug)
- `make`

### Commands

| Command | Description |
|---------|-------------|
| `make` | Compile for Arduino Nano (default) |
| `make BOARD=uno` | Compile for Arduino Uno |
| `make flash` | Flash firmware to the connected board |
| `make clean` | Remove all build artifacts |

---

## Tuning

All tuning parameters are in **`src/gimbal.h`**:

| Parameter | Default | Effect |
|-----------|---------|--------|
| `GIMBAL_KP` | `2.0` | How hard the servos correct an error. Raise for faster response, lower if oscillating |
| `GIMBAL_KI` | `0.01` | Corrects a persistent level offset. Set to `0` if the platform wobbles slowly |
| `GIMBAL_KD` | `0.1` | Damps overshoot. Raise if the platform bounces around level |
| `GIMBAL_MADGWICK_BETA` | `0.04` | Filter speed. Lower = smoother but slower to correct gyro drift |
| `GIMBAL_WARMUP_MS` | `1000` | How long (ms) the filter runs before home is captured at startup |
| `GIMBAL_MAX_DEG` | `90.0` | Maximum servo correction from centre (90° ± this value) |

**Tuning order:** set `Ki = 0`, `Kd = 0`. Raise `Kp` until it oscillates, back off 20%. Add `Kd` to stop bouncing. Add a small `Ki` only if it settles a few degrees off level.

---

## Testing & Coverage

Unit tests run on the host machine (Mac/Linux) by mocking AVR hardware registers — no hardware required.

### Prerequisites
- `gcc`
- `lcov` (`brew install lcov` on macOS)

| Command | Description |
|---------|-------------|
| `make test` | Compile and run all unit tests |
| `make coverage` | Run tests and generate coverage data |
| `make coverage-html` | Generate a visual HTML coverage report |

![Code Coverage Example](/img/code_coverage_example.png)
