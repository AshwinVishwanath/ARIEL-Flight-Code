# ARIEL Flight Code

## Overview
This repository contains the firmware for the ARIEL rocketry platform used in the University of Sheffield MACH‑25 competition entry. The code runs on a Teensy 4.1 microcontroller and integrates several sensors through an Extended Kalman Filter (EKF) for real‑time position and orientation tracking. Flight data are logged to an SD card for later analysis.

## Key Features
- **Sensor Fusion:** Combines data from an Adafruit BNO055 IMU and Adafruit BMP388 barometer.
- **Extended Kalman Filter (EKF):** Estimates 3D position and velocity at high update rates.
- **Data Logging:** Stores raw and filtered data to the SD card for post‑flight review.
- **Orientation Estimation:** Computes roll, pitch, and yaw angles using gyro integration.

## Repository Structure
```
├── flight_code.ino   # Main flight software
├── README.md         # This file
└── LICENSE           # MIT license
```

## Setup Instructions
### Prerequisites
- **Hardware:**
  - Teensy 4.1 microcontroller
  - Adafruit BNO055 IMU
  - Adafruit BMP388 barometric sensor
- **Software:**
  - Arduino IDE
  - Teensyduino plugin
  - External libraries listed below

### Installing Arduino IDE and Teensyduino
1. Download and install the [Arduino IDE](https://www.arduino.cc/en/software) for your operating system.
2. Download the Teensyduino installer from [PJRC](https://www.pjrc.com/teensy/teensyduino.html).
3. Run the Teensyduino installer and point it to your Arduino IDE installation. This adds Teensy board definitions and tools.
4. Open the Arduino IDE. Under **Tools → Board**, select **Teensy 4.1**.

### Required Libraries
Install the following libraries through the Arduino Library Manager (`Tools → Manage Libraries`) or by downloading them as ZIP archives and using `Sketch → Include Library → Add .ZIP Library`.
- **Adafruit Unified Sensor**
- **Adafruit BNO055**
- **Adafruit BMP3XX**
- **ArduinoEigen** (available via Library Manager or from the project's GitHub repository)

### Build Settings
For best performance, configure the following options in the Arduino IDE after selecting **Teensy 4.1** as the board:
1. **Tools → CPU Speed:** choose **816 MHz (overclock)**.
2. **Tools → Optimize:** choose **Fastest + LTO**.
3. Leave other settings at their defaults (USB Type: Serial, etc.).

### Uploading the Code
1. Clone or download this repository.
2. Open `flight_code.ino` in the Arduino IDE.
3. Verify that the board and port are correctly selected.
4. Click **Upload** to compile and flash the firmware onto the Teensy 4.1.

## Operation
On power‑up, the system initializes the sensors and begins logging telemetry. Output is sent to the Serial Monitor at the selected baud rate, and log files are written to the SD card.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

