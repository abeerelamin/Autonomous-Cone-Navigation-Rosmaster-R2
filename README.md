# Autonomous Cone Navigation with ROSMaster R2

Autonomous robot that detects colored cones, drives toward them, validates distance using LiDAR, and performs a random number of encirclements using IMU-based rotation tracking.

Built on the **ROSMaster R2** platform using **Python, OpenCV, LiDAR, and IMU**.

---

## 🚀 Features

- 🎨 Detects multiple cone colors (blue, green, red, orange, yellow)
- 📷 Real-time camera processing with OpenCV
- 📡 LiDAR-based obstacle / cone presence verification
- 🧭 IMU-based yaw tracking to count full encirclements
- 🔄 Random number of encirclements (1–4) per cone
- 🔀 Randomized order of cone colors
- 🧱 Modular, multi-file Python project structure

---

## 🧱 Project Structure

```text
autonomous-cone-navigation-rosmaster-r2/
│
├── main.py                # Main entry point – high-level behavior
├── vision.py              # Camera + color detection logic (OpenCV)
├── lidar.py               # LiDAR initialization and obstacle checks
├── imu_navigation.py      # Encircling logic using IMU + LiDAR
├── robot_control.py       # ROSMaster R2 hardware initialization & control
├── utils.py               # Constants and helper utilities
├── requirements.txt       # Python dependencies
├── .gitignore             # Ignore cache / environment junk
└── README.md              # This file
