# ESP32 Secure MQTT Multi-State Smart Controller

## Overview
A secure, 3-state embedded control system built on ESP32, using MQTT over TLS
to enable real-time bidirectional control and monitoring via JSON-based
publish/subscribe messaging.

The system supports control from:
- MQTT dashboards
- MQTT brokers (direct publish)
- Serial monitor (local fallback & testing)

## Features
- Secure MQTT communication (TLS, port 8883)
- 3-state finite state machine (FSM)
- JSON-based publish & subscribe control
- Bidirectional data flow (device ↔ cloud)
- OLED-based local status feedback
- Robust reconnection and state cleanup logic

## State Breakdown
- **State 1 – Sensor Mode**
  - LDR data acquisition
  - JSON payload publishing to MQTT dashboard

- **State 2 – Actuator Mode**
  - Servo motor control via MQTT(dashboard/brokers) & serial input  //
  - Slider, text, toggle & dropdown-based control support(on MQTT dashboard)

- **State 3 – Visual Feedback Mode**
  - RGB LED control via MQTT JSON payloads(brokers/dashboard)  //
  - Color picker & brightness-based control support

## MQTT Architecture
- `Tutorial_MQTT/sensors/data` → Sensor data (JSON)
- `Tutorial_MQTT/control/servo` → Servo control
- `Tutorial_MQTT/control/rgb` → RGB color control
- Dashboard-mapped subtopics (slider, toggle, dropdown)

## Hardware Used
- ESP32 DevKit
- LDR sensor
- Servo motor (SG90)
- RGB LED (common cathode)
- OLED display (SSD1306, 128×64)
- Signal booster / driver for servo control signal (3.3V GPIO → servo-compatible control)

## Documentation
-Detailed design notes, MQTT topic mapping, state transition logic, and
experimental observations for this project are available in the `/docs` directory.
-Files: - [ESP32 3-State Smart Switch MQTT Project (PDF)](ESP32_3_State_Smart_Switch_MQTT_Project.pdf)

## Status
✔ Functional implementation  
🛠 Ongoing cleanup, optimization, and documentation

## Author
Jayant Kumar  
Platform: ESP32 + Secure MQTT (TLS) + JSON
