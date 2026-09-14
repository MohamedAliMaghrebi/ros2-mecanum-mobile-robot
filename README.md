<div align="center">

# Distributed ROS 2 Control Architecture for a Four-Wheel Mecanum Mobile Robot

### Design, implementation, and experimental validation on a real omnidirectional platform

[![ROS 2](https://img.shields.io/badge/ROS%202-Distributed%20Control-22314E?style=for-the-badge&logo=ros)](https://docs.ros.org/)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%20%2B%20ESP32-A22846?style=for-the-badge)
![Validation](https://img.shields.io/badge/Validation-6%20Experimental%20Trajectories-18864B?style=for-the-badge)
![Frequency](https://img.shields.io/badge/Acquisition%20%26%20Control-50%20Hz-006D77?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Master's%20Thesis-5B4B9A?style=for-the-badge)
[![License](https://img.shields.io/badge/Code%20License-MIT-D4A017?style=for-the-badge)](LICENSE)

**[▶ View the complete experimental demonstration](https://github.com/MohamedAliMaghrebi/ros2-mecanum-mobile-robot/releases/download/v1.0.0/experimental-demonstration-ros2-mecanum-robot.mp4)**

<br>

<a href="https://github.com/MohamedAliMaghrebi/ros2-mecanum-mobile-robot/releases/download/v1.0.0/experimental-demonstration-ros2-mecanum-robot.mp4">
  <img src="docs/images/hero-platform.jpg" width="850" alt="Four-wheel Mecanum mobile robot experimental platform">
</a>

*Real four-wheel Mecanum platform developed for experimental validation.*

</div>

---

## At a Glance

| Experimental platform | Control | Validation | Best recorded indicators |
|---|---|---|---|
| 4-wheel Mecanum robot | 4 independent PI loops at 50 Hz | 6 closed trajectories | 2.34 RPM global MAE; 0.014 m estimated closure |

**Quick access:** [Demonstration](#experimental-demonstration) · [Architecture](#distributed-software-architecture) · [Control](#control-architecture) · [Results](#experimental-validation) · [Detailed panels](#detailed-experimental-performance) · [Repository structure](#repository-structure) · [Getting started](#getting-started) · [Citation](#citation)

---

## Project Highlights

- **Real experimental platform** with four independently actuated Mecanum wheels.
- **Distributed embedded architecture** combining a Raspberry Pi and an ESP32.
- **ROS 2 modular implementation** built around four specialized nodes.
- **Four independent PI wheel-speed controllers** with feedforward compensation, saturation, and anti-windup.
- **Encoder-based odometry**, automatic trajectory generation, angular correction, and web supervision.
- **Six experimentally validated closed trajectories** using one common controller configuration.
- **Full experimental traceability** through synchronized acquisition, control, and CSV logging at 50 Hz.

## Experimental Demonstration

The complete video documents the physical platform, the ROS 2 control architecture, the web-based dashboard, and the execution of rectangular, triangular, and circular trajectories.

### [Open Experimental Demonstration — Release v1.0.0](https://github.com/MohamedAliMaghrebi/ros2-mecanum-mobile-robot/releases/tag/v1.0.0)

**Video asset:** `experimental-demonstration-ros2-mecanum-robot.mp4`

---

## System Overview

The robot uses four Mecanum wheels to generate longitudinal translation, lateral translation, rotation, and combined planar motions. The architecture separates time-sensitive encoder acquisition from control, trajectory generation, odometry, supervision, and experimental recording.

<div align="center">
  <img src="docs/images/platform-and-geometry.png" width="800" alt="Mecanum robot platform and geometry">
  <br>
  <em>Mechanical platform, wheel configuration, and geometric parameters.</em>
</div>

### Hardware configuration

| Component | Function |
|---|---|
| Raspberry Pi | ROS 2 execution, control, trajectory generation, odometry, logging, and supervision |
| ESP32-WROOM | Quadrature-encoder counting, wheel-speed computation, filtering, and serial transmission |
| Four DC motors | Independent actuation of the four Mecanum wheels |
| Quadrature encoders | Wheel-speed and cumulative tick measurements |
| Motor power interface | PWM and direction commands for the four motors |
| Web dashboard | Teleoperation, trajectory execution, monitoring, reset, and emergency stop |

### Main physical and control parameters

| Parameter | Value |
|---|---:|
| Wheel radius | 0.0325 m |
| Longitudinal half-dimension, `a` | 0.175 m |
| Transverse half-dimension, `b` | 0.115 m |
| Encoder resolution | 468 ticks/revolution |
| Encoder acquisition and ROS 2 publication | 50 Hz |
| Control frequency | 50 Hz |
| Odometry update frequency | 50 Hz |
| Serial communication | 115200 bit/s |
| PI gains | Kp = 0.35, Ki = 0.45 |
| PWM saturation | ±30 |
| Angular tolerance | ±1° |

---

## Distributed Software Architecture

The final implementation uses one ESP32 firmware, four main ROS 2 nodes on the Raspberry Pi, and a dedicated Python hardware-control library. A separate validation node is provided for elementary motion tests. The ROS 2 node name and its source filename may differ; both are identified below to avoid ambiguity.

### Software modules

| Platform | Module | Main responsibility |
|---|---|---|
| ESP32 | `esp32_encoder_acquisition.ino` | Quadrature counting, RPM calculation, exponential filtering, and serial transmission |
| Raspberry Pi | `keyboard_teleop.py` — node `/keyboard_teleop_mecanum` | Manual motion-command generation |
| Raspberry Pi | `mecanum_teleop_node.py` | Inverse kinematics, feedforward action, four PI loops, saturation, and motor commands |
| Raspberry Pi | `esp32_encoder_reader.py` | Serial interface and publication of wheel RPM and cumulative ticks |
| Raspberry Pi | `mecanum_odometry_web_node.py` | Direct kinematics, calibrated odometry, trajectories, angular control, web supervision, and CSV logging |
| Raspberry Pi | `mecanum.py` | Mecanum geometry, inverse kinematics, feedforward calculation, PWM limiting, and motor-board actuation |
| Raspberry Pi | `mecanum_dashboard_validation_node.py` | Complementary validation of elementary longitudinal, lateral, and rotational commands |

### Principal ROS 2 topics

| Topic | Message type | Information |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Chassis motion command |
| `/esp32/wheel_rpm` | `std_msgs/msg/Float32MultiArray` | Measured speeds of the four wheels |
| `/esp32/wheel_ticks` | `std_msgs/msg/Int32MultiArray` | Cumulative encoder counts |

The serial interface reads and publishes encoder data every **0.02 s (50 Hz)**. Consequently, the wheel-speed regulation and measurement chain operate with a consistent 50 Hz experimental configuration.

<div align="center">
  <img src="docs/images/ros2-architecture.png" width="900" alt="ROS 2 functional architecture">
  <br>
  <em>Functional ROS 2 architecture and principal data exchanges.</em>
</div>

---

## Control Architecture

For a chassis command `[vx, vy, wz]`, inverse kinematics generates four wheel-speed references. Each wheel is regulated independently using a feedforward-plus-PI law:

```text
wheel reference → feedforward + PI correction → saturation → PWM command
                         ↑                         |
                         └──── measured speed ─────┘
```

The implementation includes:

- experimentally identified feedforward compensation;
- independent proportional-integral correction for each wheel;
- integral anti-windup;
- PWM limitation to ±30;
- synchronized wheel-speed acquisition and control at 50 Hz;
- calibrated longitudinal, transverse, and angular odometry;
- two-stage angular approach with a ±1° termination tolerance.

---

## Web-Based Supervision

<div align="center">
  <img src="docs/images/web-dashboard.png" width="900" alt="Web-based robot control and supervision dashboard">
  <br>
  <em>Interface for teleoperation, automatic trajectories, pose visualization, monitoring, reset, and emergency stop.</em>
</div>

The dashboard provides:

- manual omnidirectional teleoperation;
- automatic rectangle, triangle, and circle generation;
- real-time visualization of estimated pose and trajectory;
- wheel-speed and chassis-state monitoring;
- odometry and display reset functions;
- trajectory interruption and emergency stop;
- synchronized experimental recording.

---

## Experimental Validation

A single controller configuration was retained for the entire campaign. Six closed trajectories were executed on the real robot:

| Test | Geometry | Main solicitation |
|---|---:|---|
| R1 | Rectangle 30 × 40 cm | Translations and 90° rotations |
| R2 | Rectangle 50 × 70 cm | Translations and 90° rotations |
| R3 | Rectangle 70 × 50 cm | Translations and 90° rotations |
| T1 | Equilateral triangle, 30 cm side | Translations and 120° rotations |
| T2 | Equilateral triangle, 50 cm side | Translations and 120° rotations |
| C1 | Circle, 40 cm diameter | Simultaneous translation and rotation |

<div align="center">
  <img src="docs/images/real-experiment-50x70.jpg" width="900" alt="Real robot executing the 50 by 70 centimetre rectangle">
  <br>
  <em>Execution and supervision of the representative 50 × 70 cm rectangular trajectory.</em>
</div>

### Quantitative results

| Trajectory | Global MAE (RPM) | Maximum error (RPM) | Maximum command | Estimated odometric closure (m) | Final estimated orientation |
|---|---:|---:|---:|---:|---:|
| Rectangle 30 × 40 cm | 2.86 | 11.02 | 22 | 0.014 | 7.7° |
| Rectangle 50 × 70 cm | 2.35 | 10.95 | 22 | 0.064 | 5.6° |
| Rectangle 70 × 50 cm | **2.34** | **10.90** | 23 | 0.077 | 7.7° |
| Triangle, 30 cm side | 3.74 | 10.93 | 22 | 0.049 | 3.1° |
| Triangle, 50 cm side | 2.98 | 10.90 | 22 | 0.076 | 4.4° |
| Circle, 40 cm diameter | 4.82 | 17.90 | 25 | 0.028 | **−0.4°** |

### Key findings

- The five polygonal trajectories achieved global MAE values between **2.34 and 3.74 RPM**.
- The circular trajectory was the most demanding scenario, with a global MAE of **4.82 RPM**.
- No actuator saturation was observed: the highest command was **25**, compared with a limit of **±30**.
- The same controller configuration supported sequential translations, discrete rotations, and continuous combined motion.
- Encoder-only odometry preserved the global trajectory geometry while revealing the expected cumulative effects of Mecanum-wheel slip.

### Detailed Experimental Performance

Each panel consolidates four complementary views of one experiment: instantaneous wheel-speed tracking errors, wheel-level PI/PWM commands, wheel-speed error metrics, and closed-loop control effort. Click any panel to inspect it at full resolution.

<table>
  <tr>
    <td width="50%" align="center">
      <a href="docs/images/rectangle-30x40-performance.png"><img src="docs/images/rectangle-30x40-performance.png" alt="Four-panel performance summary for the 30 by 40 centimetre rectangle"></a><br>
      <strong>Rectangle 30 × 40 cm</strong>
    </td>
    <td width="50%" align="center">
      <a href="docs/images/rectangle-50x70-performance.png"><img src="docs/images/rectangle-50x70-performance.png" alt="Four-panel performance summary for the 50 by 70 centimetre rectangle"></a><br>
      <strong>Rectangle 50 × 70 cm</strong>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <a href="docs/images/rectangle-70x50-performance.png"><img src="docs/images/rectangle-70x50-performance.png" alt="Four-panel performance summary for the 70 by 50 centimetre rectangle"></a><br>
      <strong>Rectangle 70 × 50 cm</strong>
    </td>
    <td width="50%" align="center">
      <a href="docs/images/triangle-30-performance.png"><img src="docs/images/triangle-30-performance.png" alt="Four-panel performance summary for the 30 centimetre triangle"></a><br>
      <strong>Equilateral triangle — 30 cm</strong>
    </td>
  </tr>
  <tr>
    <td width="50%" align="center">
      <a href="docs/images/triangle-50-performance.png"><img src="docs/images/triangle-50-performance.png" alt="Four-panel performance summary for the 50 centimetre triangle"></a><br>
      <strong>Equilateral triangle — 50 cm</strong>
    </td>
    <td width="50%" align="center">
      <a href="docs/images/circle-40-performance.png"><img src="docs/images/circle-40-performance.png" alt="Four-panel performance summary for the 40 centimetre diameter circle"></a><br>
      <strong>Circle — 40 cm diameter</strong>
    </td>
  </tr>
</table>

> The transient peaks visible during direction changes correspond to reference discontinuities and wheel reversals. The panels are reported without suppressing these transients in order to preserve the traceability of the experimental observations.

<div align="center">
  <img src="docs/images/six-trajectory-comparison.png" width="900" alt="Comparison of six experimental trajectories">
  <br>
  <em>Reference paths and encoder-based odometric reconstructions for the six experimental scenarios.</em>
</div>

> **Interpretation note:** the closure and final-orientation values are internal odometric consistency indicators. They are not absolute positioning errors because no independent external localization reference was used.

---

## Repository Structure

```text
ros2-mecanum-mobile-robot/
├── README.md
├── LICENSE
├── CITATION.cff
├── .gitignore
├── software/
│   ├── esp32/
│   │   └── esp32_encoder_acquisition.ino
│   └── raspberry_pi/
│       ├── keyboard_teleop.py
│       ├── mecanum.py
│       ├── mecanum_teleop_node.py
│       ├── esp32_encoder_reader.py
│       └── mecanum_odometry_web_node.py
├── tools/
│   └── validation/
│       └── mecanum_dashboard_validation_node.py
├── config/
│   └── robot_parameters.yaml
├── data/
│   ├── README.md
│   └── experimental/
│       ├── rectangle-30x40.csv
│       ├── rectangle-50x70.csv
│       ├── rectangle-70x50.csv
│       ├── triangle-30.csv
│       ├── triangle-50.csv
│       └── circle-40.csv
└── docs/
    └── images/
        ├── hero-platform.jpg
        ├── platform-and-geometry.png
        ├── ros2-architecture.png
        ├── web-dashboard.png
        ├── real-experiment-50x70.jpg
        ├── six-trajectory-comparison.png
        ├── rectangle-30x40-performance.png
        ├── rectangle-50x70-performance.png
        ├── rectangle-70x50-performance.png
        ├── triangle-30-performance.png
        ├── triangle-50-performance.png
        └── circle-40-performance.png
```

The complete 50 Hz experimental CSV datasets used to generate the reported figures and performance indicators are available in [`data/experimental`](data/experimental). They provide traceability from the recorded wheel-level signals to the quantitative results presented in this repository.

| Dataset | Experiment |
|---|---|
| [`rectangle-30x40.csv`](data/experimental/rectangle-30x40.csv) | Rectangle 30 × 40 cm |
| [`rectangle-50x70.csv`](data/experimental/rectangle-50x70.csv) | Rectangle 50 × 70 cm |
| [`rectangle-70x50.csv`](data/experimental/rectangle-70x50.csv) | Rectangle 70 × 50 cm |
| [`triangle-30.csv`](data/experimental/triangle-30.csv) | Equilateral triangle, 30 cm side |
| [`triangle-50.csv`](data/experimental/triangle-50.csv) | Equilateral triangle, 50 cm side |
| [`circle-40.csv`](data/experimental/circle-40.csv) | Circle, 40 cm diameter |

---

## Getting Started

### Requirements

- ROS 2 on Linux;
- Python 3;
- Raspberry Pi with a USB serial connection to the ESP32;
- ESP32 firmware flashed with the encoder-acquisition program;
- four-wheel Mecanum mobile platform and compatible motor-power interface.

### Python dependencies

```bash
python3 -m pip install pyserial numpy flask
```

The ROS 2 Python packages `rclpy`, `geometry_msgs`, and `std_msgs` must be provided by the installed ROS 2 distribution.

### Serial access

The default serial interface is `/dev/ttyUSB0` at 115200 bit/s. On Linux, ensure that the current user has permission to access the serial device before starting the acquisition node.

### Execution order

```bash
# 1. Start the ESP32 serial interface
python3 software/raspberry_pi/esp32_encoder_reader.py

# 2. Start wheel control
python3 software/raspberry_pi/mecanum_teleop_node.py

# 3. Start odometry, trajectories, and web supervision
python3 software/raspberry_pi/mecanum_odometry_web_node.py

# 4. Optional manual teleoperation
python3 software/raspberry_pi/keyboard_teleop.py
```

> The commands above document the experimental startup sequence. Adapt the ROS 2 package installation, serial port, GPIO configuration, and motor-board interface to the target Raspberry Pi before execution.

### Runtime data flow

1. The ESP32 acquires the four quadrature encoders and transmits filtered wheel data over USB serial.
2. `esp32_encoder_reader.py` publishes wheel speeds and cumulative ticks at 50 Hz.
3. `mecanum_teleop_node.py` computes wheel references, applies feedforward-plus-PI control, and drives the four motors.
4. `mecanum_odometry_web_node.py` estimates the robot pose, generates trajectories, applies angular correction, and updates the web dashboard.
5. Experimental variables are recorded in CSV format for offline analysis and figure generation.

---

## Reproducibility and Data

The repository separates embedded firmware, ROS 2 control software, validated parameters, complete experimental datasets, quantitative results, and visual documentation. The six CSV recordings are published under `data/experimental` to support inspection and reproducibility.

For each published experiment, retain:

- the trajectory name and dimensions;
- the controller and odometry parameters;
- the raw CSV recording;
- the generated wheel-speed, error, PWM, and trajectory figures;
- the software revision used during the test.

---

## Academic Context

| Field | Information |
|---|---|
| Author | **Mohamed Ali Maghrebi** |
| Degree | Master's degree in Engineering — research thesis |
| Institution | Université du Québec à Rimouski (UQAR) |
| Research supervisor | Prof. Tan Sy Nguyen |
| Research areas | Mobile robotics, ROS 2, embedded systems, mechatronics, and control |
| Year | 2026 |

## Citation

If you use or refer to this work, please cite:

> M. A. Maghrebi, “Design, Implementation, and Experimental Validation of a Distributed ROS 2 Control Architecture for a Four-Wheel Mecanum Mobile Robot,” Master's thesis, Université du Québec à Rimouski, Rimouski, QC, Canada, 2026.

## License

Source code is distributed under the [MIT License](LICENSE). Photographs, diagrams, experimental results, thesis content, and video materials remain attributed to Mohamed Ali Maghrebi unless otherwise specified.

---

<div align="center">

### Mohamed Ali Maghrebi

**ROS 2 · Mobile Robotics · Mecanum Wheels · Embedded Control · Experimental Validation**

**[Experimental video](https://github.com/MohamedAliMaghrebi/ros2-mecanum-mobile-robot/releases/tag/v1.0.0)** · **[Repository](https://github.com/MohamedAliMaghrebi/ros2-mecanum-mobile-robot)**

</div>
