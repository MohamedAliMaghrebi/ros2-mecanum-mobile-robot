<div align="center">

# ROS 2 Distributed Control Architecture

## Four-Wheel Mecanum Mobile Robot

![ROS 2](https://img.shields.io/badge/ROS%202-Distributed%20Control-22314E?style=for-the-badge\&logo=ros)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%20%7C%20ESP32-C51A4A?style=for-the-badge)
![Validation](https://img.shields.io/badge/Validation-Experimental-2E8B57?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Thesis%20Project-6A5ACD?style=for-the-badge)

**Design, implementation, and experimental validation of a distributed ROS 2 control architecture for a four-wheel Mecanum omnidirectional mobile robot**

[View the experimental demonstration](https://github.com/MohamedAliMaghrebi/ros2-mecanum-mobile-robot/releases/tag/v1.0.0)

</div>

---

## Experimental Platform

<div align="center">
  <a href="https://github.com/MohamedAliMaghrebi/ros2-mecanum-mobile-robot/releases/tag/v1.0.0">
    <img src="docs/images/experimental-platform.jpg" width="720" alt="Four-wheel Mecanum experimental mobile robot">
  </a>
  <br>
  <em>Four-wheel Mecanum mobile robot developed for experimental validation.</em>
</div>

## Project Overview

This repository presents the design, implementation, and experimental validation of a distributed control architecture for a four-wheel Mecanum omnidirectional mobile robot under ROS 2.

The work integrates kinematic modeling, embedded acquisition, individual wheel-speed regulation, odometric estimation, trajectory execution, and web-based experimental supervision within a unified robotic platform.

The architecture was implemented and evaluated on a real prototype through rectangular, triangular, and circular trajectory experiments.

## Video Demonstration

The complete experimental demonstration is available through the official GitHub Release:

### [Watch or download the experimental demonstration – Version 1.0.0](https://github.com/MohamedAliMaghrebi/ros2-mecanum-mobile-robot/releases/tag/v1.0.0)

The video presents:

* the physical robotic platform;
* the distributed Raspberry Pi–ESP32 architecture;
* the ROS 2 control and supervision environment;
* individual wheel-speed tracking;
* odometric pose estimation;
* rectangular, triangular, and circular trajectories;
* experimental performance and validation results.

## System Architecture

<div align="center">
  <img src="docs/images/system-architecture.png" width="800" alt="Distributed ROS 2 control architecture">
  <br>
  <em>Functional organization of the distributed control architecture.</em>
</div>

The platform is organized around two embedded computing levels:

* **Raspberry Pi:** ROS 2 communication, high-level control, trajectory generation, wheel-speed regulation, odometric estimation, data recording, and supervision;
* **ESP32:** quadrature-encoder acquisition, wheel-speed estimation, filtering, and serial transmission;
* **Motor-power interface:** independent actuation of the four DC motors;
* **Web interface:** teleoperation, parameter configuration, monitoring, and emergency-stop functions.

## Control Strategy

The proposed control architecture combines:

* inverse kinematics for wheel-speed reference generation;
* four independent PI wheel-speed controllers;
* feedforward compensation of motor dead zones;
* saturation and anti-windup protection;
* direct kinematics and calibrated encoder-based odometry;
* angular correction during trajectory execution;
* multirate communication between control and measurement processes.

## Supervision Interface

<div align="center">
  <img src="docs/images/web-dashboard.png" width="800" alt="ROS 2 web-based supervision interface">
  <br>
  <em>Web-based interface used for teleoperation, monitoring, and experimental supervision.</em>
</div>

The supervision interface provides access to the motion commands, controller parameters, encoder measurements, wheel-speed references, measured velocities, PWM commands, odometric pose, and experimental recording functions.

## Experimental Validation

The platform was evaluated using six closed trajectories:

| Trajectory              |         Dimensions |
| ----------------------- | -----------------: |
| Rectangle R1            |         30 × 40 cm |
| Rectangle R2            |         50 × 70 cm |
| Rectangle R3            |         70 × 50 cm |
| Equilateral triangle T1 | Side length: 30 cm |
| Equilateral triangle T2 | Side length: 50 cm |
| Circle C1               |    Diameter: 40 cm |

<div align="center">
  <img src="docs/images/trajectory-results.png" width="850" alt="Experimental trajectory validation results">
  <br>
  <em>Representative experimental results obtained for the validated trajectories.</em>
</div>

The experimental campaign evaluates wheel-speed tracking, control effort, odometric consistency, trajectory execution, and the practical effects of Mecanum-wheel slippage.

## Repository Organization

```text
ros2_ws/src/   ROS 2 packages and control nodes
esp32/         Encoder acquisition and embedded measurement code
scripts/       Experimental data processing and figure generation
config/        Robot and controller configuration files
docs/images/   Photographs, architecture diagrams, and result figures
```

## Academic Context

| Information         | Details                                                             |
| ------------------- | ------------------------------------------------------------------- |
| Author              | Mohamed Ali Maghrebi                                                |
| Institution         | Université du Québec à Rimouski                                     |
| Program             | Master’s Degree in Engineering                                      |
| Research supervisor | Prof. Tan Sy Nguyen                                                 |
| Research fields     | Mobile robotics, ROS 2, embedded systems, mechatronics, and control |
| Year                | 2026                                                                |

## Citation

If you use or refer to this work, please cite:

> M. A. Maghrebi, “Design, Implementation, and Experimental Validation of a Distributed ROS 2 Control Architecture for a Four-Wheel Mecanum Mobile Robot,” Master’s thesis, Université du Québec à Rimouski, Rimouski, QC, Canada, 2026.

## Author

**Mohamed Ali Maghrebi**
Master’s student in Engineering
Université du Québec à Rimouski
Rimouski, Québec, Canada

## License

The source code in this repository is distributed under the MIT License. Photographs, diagrams, experimental results, and video materials remain attributed to Mohamed Ali Maghrebi unless otherwise specified.

---

<div align="center">

**ROS 2 · Mobile Robotics · Mecanum Wheels · Embedded Control · Experimental Validation**

</div>
