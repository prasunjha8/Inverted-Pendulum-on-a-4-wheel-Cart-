# Inverted-Pendulum-on-a-4-wheel-Cart-

# Inverted Pendulum on a 4-Wheel Cart (Self-Balancing Robot)

Welcome to the GitHub repository of my self-balancing robot project! This robot uses a 4-wheel cart to balance an inverted pendulum (vertical rod) in real-time using sensor feedback and PID control, implemented on an ESP8266 microcontroller.

---

## Project Overview

This project aims to demonstrate real-time control of an **inverted pendulum**, a classical problem in dynamics and control systems. The system detects the tilt of the pendulum and adjusts the wheel motors to keep it upright—just like how we instinctively balance a broom on our hand.

---

## Concept: Inverted Pendulum

An inverted pendulum is inherently unstable. When the rod tilts, gravity causes it to fall further unless actively corrected. The robot continuously senses the angle and applies corrective motion using PID (Proportional-Integral-Derivative) control to stabilize the system.  

In our setup, the cart moves forward or backward based on the direction of tilt, so that the base stays under the center of mass.

---

## Components Used

| Component             | Description                                    |
|----------------------|------------------------------------------------|
| **ESP8266 NodeMCU**   | Microcontroller with Wi-Fi support            |
| **MPU6050**           | IMU sensor (Accelerometer + Gyroscope)        |
| **BO Motors (4x)**    | 1:120 gear ratio DC motors                    |
| **Motor Driver**      | L298N Dual H-Bridge driver                    |
| **12V Power Supply**  | Power source for motors                       |
| **Rod**               | Acts as the inverted pendulum                 |

---

## How It Works

1. **Sensor Reading**: MPU6050 provides raw angle data (accelerometer + gyro).
2. **Angle Estimation**: A Kalman filter fuses accelerometer and gyroscope data to get accurate angle readings.
3. **PID Control**: A PID controller calculates the required motor speed based on the current angle error.
4. **Motor Output**: The motor driver receives PWM signals to move the cart accordingly.
5. **Real-Time Tuning**: A Wi-Fi-based web interface is used to tune PID constants on the fly.

---


