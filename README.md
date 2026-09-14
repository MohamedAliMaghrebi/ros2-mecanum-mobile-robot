# ROS 2 Control Architecture for a Mecanum Mobile Robot

## Overview

This repository presents the design, implementation, and experimental validation of a distributed control architecture for a four-wheel Mecanum omnidirectional mobile robot under ROS 2.

The work was conducted as part of a master’s thesis in engineering at the Université du Québec à Rimouski (UQAR). It focuses on the integration of kinematic modeling, embedded control, wheel-speed regulation, odometric estimation, trajectory execution, and experimental supervision within a unified robotic platform.

## Experimental Demonstration

The complete experimental demonstration is available through the following GitHub Release:

### [Watch or download the experimental demonstration](https://github.com/MohamedAliMaghrebi/ros2-mecanum-mobile-robot/releases/tag/v1.0.0)

The video presents the physical platform, the ROS 2 supervision interface, individual wheel-speed control, odometric monitoring, and the execution of rectangular, triangular, and circular trajectories.

## System Architecture

The control architecture is distributed between:

* a **Raspberry Pi**, responsible for ROS 2 communication, high-level control, trajectory generation, wheel-speed regulation, odometric estimation, and supervision;
* an **ESP32**, dedicated to quadrature-encoder acquisition and measured wheel-speed transmission;
* a motor-power interface controlling four independent DC motors;
* a web-based interface providing teleoperation, monitoring, parameter adjustment, and emergency-stop functions.

## Control Strategy

The proposed strategy combines:

* inverse kinematics for individual wheel-speed reference generation;
* four independent PI wheel-speed controllers;
* feedforward compensation of actuator dead zones;
* output saturation and anti-windup protection;
* direct kinematics and calibrated encoder-based odometry;
* angular correction during trajectory execution.

## Experimental Validation

The platform was evaluated through six closed trajectories:

* three rectangular trajectories: 30 × 40 cm, 50 × 70 cm, and 70 × 50 cm;
* two equilateral triangular trajectories with side lengths of 30 cm and 50 cm;
* one circular trajectory with a diameter of 40 cm.

The experiments assess individual wheel-speed tracking, control effort, odometric consistency, trajectory execution, and the practical limitations associated with Mecanum-wheel slippage.

## Academic Context

* **Author:** Mohamed Ali Maghrebi
* **Institution:** Université du Québec à Rimouski
* **Program:** Master’s Degree in Engineering
* **Research supervisor:** Prof. Tan Sy Nguyen
* **Research fields:** Mobile robotics, ROS 2, embedded systems, mechatronics, and control
* **Year:** 2026

## Citation

If you use or refer to this work, please cite:

> M. A. Maghrebi, “Design, Implementation, and Experimental Validation of a Distributed ROS 2 Control Architecture for a Four-Wheel Mecanum Mobile Robot,” Master’s thesis, Université du Québec à Rimouski, Rimouski, QC, Canada, 2026.

## License

The source code made available in this repository is distributed under the MIT License. The experimental video and associated documentation remain attributed to Mohamed Ali Maghrebi.
