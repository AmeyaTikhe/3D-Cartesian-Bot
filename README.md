# 3D-Cartesian-Bot

This project implements a 3D Cartesian robotic system controlled by an ESP32 microcontroller.
It utilizes the ESP32's RMT (Remote Control) peripheral to generate precise stepper motor control signals for 3 NEMA17 motors, enabling smooth motion profiles with acceleration, constant speed, and deceleration phases.
The system is designed for applications requiring precise linear movements, specifically to replicate the motion of surgical incisions used in real-life medical procedures.

The system uses OpenCV to recognize dots on paper, transferring the coordinates to a web interface and then to the ESP32 for controlling stepper motors. The ESP32 posts log statements on the ESP-IDF terminal to simulate motor movements between coordinate sets, replicating the transition between points accurately.

---

## Features

- **Smooth Stepper Motor Control**: Generates stepper motor signals with acceleration and deceleration phases using the ESP32's RMT peripheral.
- **OpenCV Integration**: Utilizes OpenCV for computer vision tasks, potentially enabling features like object detection or path planning.
- **WebSocket Frontend**: Provides a frontend for user interaction and control.

---

# Simulation


---

## Future Work

- Design a custom PCB to connect three NEMA17 stepper motors, compatible with a customized Creality Ender 3 V2 3D printer assembly.
- Implement machine learning to recognize dot patterns and categorize them into different types of surgical incisions, allowing for path planning based on incision type.
- Simulate the process by actuating a pen to connect dots drawn on a reusable sheet.
- Incorporate the ability to write at an angle, replicating the angled characteristic of a surgical incision.

---

### Prerequisites

- ESP32 development board.
- ESP-IDF development environment set up on your system.
