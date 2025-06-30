# Stabilite

> An ESP32-powered assistive smart spoon with real-time active tremor stabilization for individuals with Parkinson's disease.

---


## Overview

For individuals living with Parkinson's disease and Essential Tremor (ET), involuntary hand tremors make everyday tasks—especially eating independently—challenging and frustrating. Spills and loss of utensil control often lead to reduced independence and social discomfort during meals.

**Stabilite** is an open, low-cost assistive smart spoon built around the ESP32 microcontroller. Using high-frequency 6-DoF inertial sensing from an MPU-9250 IMU, Stabilite continuously measures hand tremor vibrations, calculates roll and pitch angle deviations via a complementary sensor fusion filter, and dynamically actuates dual micro-servos to counter-rotate the spoon head in real time. This keeps the spoon bowl level and stable, preventing food spillage and restoring dining independence.

---

## Features

- **Active Dual-Axis Servo Stabilization**: Compensates for both roll (lateral tilt) and pitch (forward/backward tilt) in real time.
- **Sensor Fusion (Kalman-Style Filtering)**: Combines gyroscope angular velocity integration with accelerometer gravitational vectors ($\alpha = 0.96$) to eliminate gyro drift and high-frequency noise.
- **Automatic Gyroscope Calibration**: Samples 200 IMU readings at startup to compute zero-motion offset biases for $X$, $Y$, and $Z$ axes.
- **Real-Time Motion Classification**: Detects active tremor states vs. calm/idle resting states using angular velocity thresholds.
- **Visual RGB LED Feedback**: Multicolored state feedback indicating calibration, excessive motion warnings, active stabilization, and idle modes.
- **Serial Telemetry Diagnostics**: Real-time logging of pitch/roll angles and servo compensation pulses over UART at 115200 baud.

---

## Hardware (Bill of Materials)

| Component | Quantity | Purpose & Specifications |
|---|:---:|---|
| **ESP32 Development Board** | 1 | Main microcontroller (Dual-core 240 MHz, 3.3V logic, hardware PWM & I2C) |
| **MPU-9250 (or MPU-6050) IMU** | 1 | 6-Axis motion tracking (3-axis gyroscope $\pm 500^\circ/s$, 3-axis accelerometer $\pm 4g$) |
| **Micro Servo Motors (SG90 / MG90S)** | 2 | Dual-axis physical counter-tilt actuation (Roll on GPIO 17, Pitch on GPIO 16) |
| **Common Cathode RGB LED** | 1 | Visual diagnostic indicator (PWM brightness control for Red, Green, Blue) |
| **Current-Limiting Resistors (220Ω - 330Ω)** | 3 | Protects RGB LED channels from overcurrent |
| **Power Source (3.7V - 5V LiPo / Battery Pack)**| 1 | Powers ESP32 board and servo motor rails |
| **Prototyping Board / 3D-Printed Chassis** | 1 | Lightweight handheld handle and pivoting spoon gimbal |

---

## How It Works

Stabilite operates on a 50 Hz (~20 ms period) real-time control loop:

```
┌─────────────────────────────────────────────────────────┐
│                      MPU-9250 IMU                       │
│     (Raw Accel: ax, ay, az  |  Raw Gyro: gx, gy, gz)    │
└────────────────────────────┬────────────────────────────┘
                             │ I2C (Wire on GPIO 21/22)
                             ▼
┌─────────────────────────────────────────────────────────┐
│                   ESP32 Microcontroller                 │
│                                                         │
│  1. Subtract Zero-Motion Gyro Offsets                   │
│  2. Compute Accel Tilt:                                 │
│     - accPitch = atan2(-ax, sqrt(ay² + az²)) * 180 / π  │
│     - accRoll  = atan2(ay, az) * 180 / π                │
│                                                         │
│  3. Complementary Sensor Fusion Filter:                 │
│     - rollAngle  = 0.96*(rollAngle + gy*dt) + 0.04*accP │
│     - pitchAngle = 0.96*(pitchAngle + gx*dt) + 0.04*accR│
│                                                         │
│  4. Motion Classification (Tremor vs Idle)              │
│  5. Inverse Compensation Calculation                    │
└──────────────┬───────────────────────────┬──────────────┘
               │                           │
               ▼ PWM                       ▼ PWM
┌─────────────────────────────┐ ┌─────────────────────────┐
│      Dual Micro-Servos      │ │   Common Cathode RGB    │
│  Roll:  90° - (rollAngle)   │ │  Yellow: Calibrating    │
│  Pitch: 90° + (pitchAngle)  │ │  Green:  Tremor Active  │
│  Range: 0° – 180°           │ │  Blue:   Calm / Idle    │
└─────────────────────────────┘ └─────────────────────────┘
```

1. **Startup Calibration**: When powered on, the firmware reads 200 consecutive IMU samples to calculate zero-rate offsets (`gyroXoffset`, `gyroYoffset`, `gyroZoffset`) while indicating a **Yellow LED**. If motion exceeds $7.0^\circ/s$ during startup, the LED shifts to **Red** to prompt the user to keep the handle steady.
2. **Complementary Angle Estimation**: Raw accelerometer readings are converted to Euler pitch and roll angles via trigonometric projections (`atan2`). The complementary filter fuses high-frequency gyro rate integration with low-frequency gravity vectors to produce a noise-free, drift-compensated angle.
3. **Counter-Actuation**: Calculated angle deviations are inverted and scaled by `COMPENSATION_GAIN` to generate target servo angles constrained within safe mechanical limits ($0^\circ–180^\circ$ centered at $90^\circ$).
4. **Motion State Indication**:
   - **Green LED**: Active hand tremor detected ($\|\vec{\omega}\| > 3.0^\circ/s$).
   - **Blue LED**: Steady / calm state (no tremor detected for $> 1.5$ seconds).

---

## Circuit & Wiring

### Pinout Configuration

| Component Pin | ESP32 GPIO | Description / Notes |
|---|:---:|---|
| **MPU-9250 SDA** | `GPIO 21` | I2C Data line |
| **MPU-9250 SCL** | `GPIO 22` | I2C Clock line |
| **MPU-9250 VCC / GND** | `3.3V / GND` | Sensor power rail |
| **Roll Servo Signal** | `GPIO 17` | PWM control for horizontal axis |
| **Pitch Servo Signal** | `GPIO 16` | PWM control for vertical axis |
| **Servos Power / GND** | `5V (VIN) / GND` | Servo power rail (external power recommended) |
| **RGB LED Red** | `GPIO 12` | Active low PWM channel (Common Cathode) |
| **RGB LED Green** | `GPIO 13` | Active low PWM channel (Common Cathode) |
| **RGB LED Blue** | `GPIO 27` | Active low PWM channel (Common Cathode) |
| **RGB Common Cathode** | `GND` | Ground rail |

### Hardware Layout

![Stabilite Hardware Layout](images/Parkinsons%20Spoon%20Labelled%20Layout.jpeg)

---

## Active Stabilization Demonstration

| Pitch Up Compensation | Pitch Down Compensation | Lateral Roll Compensation |
|:---:|:---:|:---:|
| ![Stabilizing Up](images/Stabilizing%20when%20moved%20up.jpeg) | ![Stabilizing Down](images/Stabilizing%20when%20moved%20down.jpeg) | ![Stabilizing Right](images/Stabilizing%20when%20moved%20right.jpeg) |
| *Counter-tilting downwards when handle pitches upward* | *Counter-tilting upwards when handle pitches downward* | *Counter-rotating when handle rolls right* |

---

## Setup & Flashing Instructions

### 1. Requirements & Dependencies

- **Arduino IDE** (version 2.x recommended) or **VS Code with PlatformIO**
- **ESP32 Board Package**:
  1. Open Arduino IDE $\rightarrow$ **File** $\rightarrow$ **Preferences**.
  2. Add the following URL to *Additional Boards Manager URLs*:
     ```text
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
  3. Go to **Tools** $\rightarrow$ **Board** $\rightarrow$ **Boards Manager**, search for `esp32`, and install the official package by Espressif Systems.
- **Required Libraries**:
  - `ESP32Servo` (Install via Arduino Library Manager by Kevin Harrington / John K. Bennett)
  - `Wire` (Built into ESP32 Arduino core)

---

### 2. Flashing the Firmware

1. Open [`stabilite_firmware.ino`](stabilite_firmware.ino) in Arduino IDE.
2. Select your board under **Tools** $\rightarrow$ **Board** $\rightarrow$ **ESP32 Arduino** $\rightarrow$ **ESP32 Dev Module** (or your specific ESP32 variant).
3. Connect your ESP32 board via USB and select the appropriate COM port (**Tools** $\rightarrow$ **Port**).
4. Click **Upload** ($\rightarrow$).
5. Open the **Serial Monitor** at **115200 baud** to view calibration output and live angle telemetry.

---

## Future Improvements

- **Adaptive Gain Tuning**: Implement machine-learning or auto-tuning PID control to dynamically adjust compensation gains to individual tremor frequencies (typically 4–8 Hz).
- **Custom 3D-Printed Ergonomic Enclosure**: Design a waterproof, washable snap-on food-grade utensil head with an internal battery compartment.
- **Bluetooth / Wi-Fi Telemetry App**: Sync tremor frequency and amplitude data to a companion mobile dashboard to help neurologists track disease progression over time.

---

## License

This project is open source and available under the [MIT License](LICENSE).
