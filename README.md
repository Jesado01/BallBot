# 🤖 Intelligent Self-Balancing Ballbot

This project implements a **neural network-based PID control system** for a self-balancing robotic platform. It combines traditional control theory with machine learning to dynamically adjust control parameters in real time, improving stability, adaptability, and performance under nonlinear and unstable conditions.

---

## 📌 Project Summary

- **Goal:** Improve the stability and responsiveness of a self-balancing platform by using a trained neural network to adaptively tune PID gains (Kp, Ki, Kd) in real time.
- **Hardware:** Jetson Nano, Arduino Mega 2560, MPU6050 IMU, DC motors with encoders.
- **Software:** Python, C++, TensorFlow, NumPy, Pandas, Matplotlib.

---

## 📸 Demo

*Coming soon: video and GIFs showing the ballbot balancing and reacting to external disturbances.*

---

## 🔧 System Architecture

- Arduino reads IMU data and controls motors via PWM.
- Jetson Nano runs a trained neural network that predicts optimal PID gains using live sensor input.
- PID values are sent from Jetson to Arduino via serial communication.
- Arduino applies motor corrections in real-time.
