# IoT-Based Proximity Detection System

## Overview
This project implements a proximity detection system using an ESP32 microcontroller, ultrasonic sensors, and a buzzer. The system provides real-time distance monitoring, triggers alerts when objects move out of a defined "buffer zone," and offers a web interface for system control and data visualization.

## Features
- **Real-Time Proximity Monitoring**: Utilizes an ultrasonic sensor to measure distance and log data.
- **Buffer Zone Alerts**: Triggers a buzzer if the detected distance is outside the configured range (e.g., 30–60 cm).
- **Web Interface**:
  - Control system settings such as buzzer volume and frequency.
  - View real-time distance measurements.
  - Access a visual graph of historical distance data.
- **Communication Protocols**: Implements the I2C protocol for sensor communication and HTTP for web interface interaction.

## Hardware Requirements
- **ESP32 Microcontroller**
- **Ultrasonic Sensor** (e.g., HC-SR04)
- **Buzzer**
- Power supply and basic wiring components

## Software Requirements
- **ESP-IDF Framework**
- FreeRTOS (included in ESP-IDF)
- Basic understanding of C programming

## Setup Instructions
1. **Hardware Setup**:
   - Connect the ultrasonic sensor to the ESP32 using the appropriate GPIO pins (e.g., `TRIG` on GPIO 23, `ECHO` on GPIO 21).
   - Connect the buzzer to GPIO 22.
   - Ensure all components are powered properly.

2. **Code Configuration**:
   - Clone this repository and open the project in an ESP-IDF-compatible IDE.
   - Update the WiFi credentials in the code:
     ```c
     #define EXAMPLE_ESP_WIFI_SSID "YourWiFiSSID"
     #define EXAMPLE_ESP_WIFI_PASS "YourWiFiPassword"
     ```
   - Build and flash the code onto the ESP32.

3. **Run the System**:
   - Connect to the ESP32's IP address using a web browser to access the control interface.
   - View real-time measurements and configure settings.

## How It Works
- The system periodically measures distance using the ultrasonic sensor.
- Data is processed in real-time, and alerts are triggered if the object is outside the "buffer zone."
- Measurements are logged and displayed on the web interface for visualization and control.

## Web Interface Features
- **Real-Time Data**: Displays current distance measurements.
- **Historical Data**: Plots distance over time for analysis.
- **System Controls**:
  - Arm/Disarm the alert system.
  - Adjust buzzer frequency and volume dynamically.

## Technical Details
- **Programming Language**: C
- **Key Libraries/Frameworks**:
  - ESP-IDF
  - FreeRTOS
- **Communication Protocols**:
  - I2C for sensor communication
  - HTTP for web interface interaction
- **Data Logging**:
  - Stores up to 6000 distance measurements in memory for analysis.

## Possible Extensions
- Integration with cloud platforms for remote monitoring.
- Use of advanced sensors like LiDAR for greater accuracy.
- Mobile app interface for easier control and monitoring.

## License
This project is licensed under the MIT License. See `LICENSE` for details.

## Acknowledgments
- Developed using the ESP-IDF framework and FreeRTOS.
- Inspired by IoT applications for real-time monitoring and control.
