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

| Metric                           | PID                     | PIDNN                   |
|----------------------------------|--------------------------|--------------------------|
| **Static Conditions**            |                          |                          |
| Overshoot (%)                   | 10%                     | 3%                      |
| Steady-State Error (°)          | ±1.8                    | ±0.3                    |
| Settling Time (s)               | 2.8                     | 1.2                     |
| Max PWM                         | ±150                    | ±60                     |
| General Stability               | Moderate oscillations   | High stability          |
| Adaptation to System Changes    | Slow                    | Fast and stable         |
| Computational Complexity        | Low                     | Medium                  |
| Robustness (Static)             | Limited                 | High                    |
|                                  |                          |                          |
| **Under External Disturbance**  |                          |                          |
| Overshoot (%)                   | 15%                     | 5%                      |
| Steady-State Error (°)          | ±2.5                    | ±0.5                    |
| Settling Time (s)               | 3.2                     | 1.5                     |
| Max PWM                         | ±230                    | ±80                     |
| General Stability               | Persistent oscillations | High stability          |
| Adaptation to Disturbances      | Slow                    | Fast and precise        |
| Energy Efficiency               | Medium                  | High                    |

---

## 🧠 Technologies Used

- Classical PID (manual tuning + Ziegler-Nichols method)
- Neural networks trained via supervised learning
- Jetson Nano for real-time inference
- Serial communication between Arduino and Jetson

---

## 🙋‍♂️ Author

**Jesús Adolfo Coronado Brenes**  
👨‍🎓 Mechatronics Engineering - UACJ  
📧 adolfo.coronado05@gmail.com

---

## 📄 License

MIT License – Free to use with proper attribution 🙌
