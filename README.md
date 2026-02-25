🤖 Color Sorting Robotic Arm

Autonomous ESP8266-powered 5-DOF robotic arm that detects and sorts objects by color using a TCS3200 sensor, featuring a real-time web dashboard and persistent configuration storage.

📌 Overview

This project implements a WiFi-enabled autonomous color sorting system using an ESP8266 microcontroller. The robotic arm detects object color (Red, Green, Blue) and places them into designated bins using a structured Finite State Machine (FSM) architecture.

The system is fully controllable through a built-in web interface hosted directly on the ESP8266 (Access Point mode — no router required).

🔹 Core Components

ESP8266 (NodeMCU) – Main controller

PCA9685 (I2C PWM Driver) – 16-channel servo control

5x Servo Motors – 5-DOF robotic arm

TCS3200 Color Sensor – RGB color detection

IR Sensor – Object presence detection

Relay Module – Conveyor belt control

✨ Key Features
🎯 Autonomous Sorting

Detects Red, Green, Blue

Automatically executes bin placement sequence

🌐 Web Dashboard (192.168.4.1)

Start / Pause sorting

Real-time counters (R/G/B/Total)

Conveyor status indicator

Last detected color

Auto-refresh (500ms)

🎛 Manual Control Page (/control)

Individual servo angle sliders (0–180°)

Smooth motion algorithm

Grab-point configuration

Relay off-delay adjustment

EEPROM Save/Load

💾 Persistent Storage

Stores 6 grab positions

Stores relay timing

EEPROM validation using magic number 0xA55A12EF

Survives power cycles

🚀 Getting Started
1️⃣ Flash Firmware

Upload using Arduino IDE.

Required Libraries:

ESP8266WiFi

ESP8266WebServer

Adafruit_PWMServoDriver

Wire

EEPROM

2️⃣ Power On

Connect via USB or external supply.

3️⃣ Connect to WiFi

SSID: ColorSort-RobotArm
Password: 12345678

4️⃣ Open Web Interface
http://192.168.4.1

5️⃣ Configure Grab Points

Navigate to /control and adjust pickup positions.

Press Start to begin sorting.

⚙ Calibration Notes

Servo pulse range: 150–600 ticks

Gripper servo: 150–630 ticks

TCS3200 scaling: 20% frequency

Relay off-delay prevents mechanical chatter

🧠 Engineering Highlights

Modular FSM-based architecture

Smooth servo interpolation algorithm

I2C-based multi-servo control

Embedded web server (no external backend)

EEPROM-based configuration persistence

Access Point networking mode

🔮 Future Improvements

Additional color bins (Yellow, Black, White)

Machine learning-based color recognition

Limit switches for homing

Mobile application control

OTA firmware updates

Cloud-based data logging

📜 License

MIT License — Free to use, modify, and distribute.

👨‍💻 Author

Achira Pamuditha and thineth nirmal
Software Engineering Student | Embedded Systems Enthusiast
