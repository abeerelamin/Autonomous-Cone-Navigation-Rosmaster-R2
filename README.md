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
## Technical Design

### 1. Vision-Based Object Detection
The system uses HSV color segmentation to detect cones in real-time.  
The largest contour matching the target color is selected to reduce noise.  
Bounding box area is used as a proximity heuristic.

### 2. Servo-Based Alignment
The horizontal center of the detected cone is mapped to a 0–180° servo angle:
angle = (center_x / frame_width) * 180

This keeps the robot aligned while approaching the cone.

### 3. LiDAR Verification
Before initiating encirclement, a narrow LiDAR angular window (85°–90°)  
is scanned to confirm the presence of an obstacle within a threshold distance.

This prevents false positives from vision alone.

### 4. IMU-Based Encirclement Tracking
Yaw readings are normalized to [0, 360°].
Rotation difference is computed using shortest-angle logic.

Each encirclement is detected when yaw difference reaches ~160°–164°,
with cooldown filtering to avoid false loop detection.

This approach avoids wheel encoder dependency.

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


