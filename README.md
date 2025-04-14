# Self-Balancing Platform with Intelligent Control using Neural Networks

This repository contains the code, documentation, and implementation details of a self-balancing robotic platform that uses intelligent control based on deep learning and artificial neural networks. The project compares the performance of a classical PID controller with a neural network-enhanced PID (PIDNN) to evaluate the improvement in stability, responsiveness, and disturbance rejection.

## 📌 Project Overview

Traditional PID controllers are widely used in industrial applications due to their simplicity and efficiency. However, they struggle in nonlinear systems and dynamic environments. This project integrates a neural network to dynamically tune the PID gains (`Kp`, `Ki`, `Kd`) in real time, providing greater adaptability and improved performance in controlling an unstable, multivariable inverted pendulum system.

## 🧠 Intelligent Control System

- **Controller Type**: PID and Neural Network-based PID (PIDNN)
- **Neural Network**: Feedforward NN with ReLU activation
- **Inputs**: Error `e(t)`, its derivative, and integral
- **Outputs**: `Kp`, `Ki`, `Kd` (dynamically adjusted every control loop)
- **Training**: Supervised learning using real-world data from the platform
- **Hardware**: Jetson Nano + Arduino Mega 2560 + MPU6050 IMU + DC motors

## ⚙️ System Components

- **Microcontroller**: Arduino Mega 2560 (low-level motor control)
- **Sensor**: MPU6050 (pitch & roll measurement)
- **Compute Unit**: Jetson Nano (runs neural network in real time)
- **Motors**: JGA25-370 DC motors with encoders
- **Driver**: L298N motor driver
- **Power**: LiPo Battery
- **Chassis**: Custom CAD-designed body with omnidirectional wheels

## 🧪 Experimental Setup

The system was tested under various conditions to assess:
- Static balancing performance
- Response to external disturbances
- Settling time, overshoot, and steady-state error
- Comparison between classical PID vs PIDNN

## 📊 Results

The neural network-enhanced controller outperformed the classical PID by:
- Reducing steady-state error
- Improving response time and stability
- Demonstrating greater robustness to system disturbances

A full analysis of these results is presented in the [scientific article](./docs/Articulo_cientifico.pdf) and [thesis document](./docs/TESIS_CORONADO.pdf).

## 🗂️ Repository Structure

```plaintext
├── arduino/
│   ├── PID_Controller/
│   └── PIDNN_Controller/
├── python/
│   └── NeuralNetwork_Model/
├── docs/
│   ├── Articulo_cientifico.pdf
│   └── TESIS_CORONADO.pdf
├── models/
│   └── trained_nn_model.h5
├── cad/
│   ├── Platform_CAD_Files/
│   └── Assembly_Images/
└── README.md
