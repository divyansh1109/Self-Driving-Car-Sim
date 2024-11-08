# Self Driving Car Sim

A self-driving car simulation in a Gazebo environment using ROS2, OpenCV, and custom machine learning models. This project enables autonomous driving by detecting and responding to lanes, signboards, and traffic lights.

---

## Overview

This project simulates a self-driving car in a Gazebo environment that includes lanes, traffic signals, and traffic lights. Using a combination of classical and advanced computer vision techniques, as well as custom-trained deep learning models, the car can detect its surroundings and make driving decisions accordingly.

**Core Objectives:**
- Lane detection and following
- Signboard detection for speed limits, turns, and stops
- Traffic light detection and response

## Features

### Lane Detection
- **Method**: Lane segmentation with classical computer vision.
- **Process**:
  1. White and yellow lane markings are segmented from the image.
  2. The region of interest (ROI) is extracted.
  3. Image processing techniques are applied to determine lane curvature and position relative to the center.
  4. This data is used to control the car’s lateral movement to stay centered in the lane.

### Signboard Detection and Tracking
- **Method**: Custom-trained CNN model for sign detection and Optical Flow for tracking.
- **Process**:
  1. The CNN model is trained to recognize speed limits (30, 60, 90), turn indicators, and stop signs.
  2. Once detected, Optical Flow tracks the signs to reduce computation and maintain accuracy.
  3. The car adjusts its speed or makes necessary turns based on the detected sign.

### Traffic Light Detection
- **Method**: Haar Cascade for detection and Optical Flow for tracking.
- **Process**:
  1. Haar Cascade identifies the traffic light location and color (green or red).
  2. Optical Flow helps track the traffic light to maintain detection.
  3. The car stops on red and moves on green.

## Technologies Used

- **ROS2**: For simulation and communication between modules.
- **Gazebo**: 3D simulation environment.
- **OpenCV**: Classical and advanced computer vision techniques for lane, sign, and light detection.
- **Convolutional Neural Networks (CNN)**: Custom-trained model for detecting various signboards.
- **Optical Flow**: Advanced CV technique for tracking detected objects.

## Future Improvements

- Integrate additional traffic signs and signals.
- Improve the robustness of lane detection and control avoiding the ambiguity of Curvature.
- Experiment with other machine learning models to increase accuracy and performance.

---
