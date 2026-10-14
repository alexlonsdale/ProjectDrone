# ESP32 Autonomous Quadcopter Platform

## Overview
This repository contains the hardware specifications, mechanical CAD files, and custom C++ flight control stack for an ESP32-based autonomous quadcopter. The project focuses on a scratch-built, dual-power guidance and control system designed to execute basic spatial tasks (auto-hover, directional translation) prior to the integration of a custom RF RC interface. 

The physical chassis is designed in SolidWorks and optimized for rapid prototyping on a Bambu Lab A1 Mini (180x180x180mm build volume), targeting a sub-250mm wheelbase.

## System Architecture

The platform architecture is divided into three primary domains: the internal Guidance and Control Unit (GCU), the propulsion loop, and the physical chassis. To prevent microcontroller brownouts from high-current motor draws, the system utilizes a strictly isolated dual-power architecture.

### Hardware Specifications

| Component | Specification | Function |
| :--- | :--- | :--- |
| **Microcontroller** | ESP32 | GCU core; runs the C++ flight stack, handles kinematics, PWM generation, and state machine. |
| **IMU** | MPU6500 | 6-axis Gyroscope/Accelerometer; feeds attitude and acceleration data to the GCU. |
| **ESC** | 4-in-1 60A ESC | Drives the brushless motors via PWM signals from the ESP32. |
| **Motors** | 4x Brushless (Specs TBD) | Paired with 5-inch propellers for high-thrust maneuverability. |
| **Motor Power** | 6S Li-Po | High-voltage primary loop dedicated exclusively to the ESC and motors. |
| **GCU Power** | Secondary Battery | Isolated low-voltage power supply dedicated to the ESP32 and MPU6500 to ensure logic stability. |

### Software & Control Systems

The flight controller is being developed entirely from scratch in C++. 

* **Guidance Program:** An internal state machine dictates basic autonomous behaviors. The system is currently capable of localized spatial tasks, utilizing internal setpoints for hovering and vector translations (up, down, left, right).
* **Control Loop:** Attitude and altitude stabilization are managed by a custom Proportional-Integral-Derivative (PID) controller. 
* **Modeling & Tuning:** Early-stage control loops and system dynamics can be simulated and tuned using MATLAB/Simulink prior to flashing the C++ stack to the ESP32, ensuring theoretical stability before physical flight tests.
* **Sensor Fusion:** Raw MPU6500 data (accelerometer and gyroscope) is filtered and fused to calculate accurate roll, pitch, and yaw estimates.

### Mechanical Design

* **CAD Software:** SolidWorks
* **Configuration:** Quad-X frame (Targeting ~200-240mm wheelbase).
* **Manufacturing:** 3D printed components. The geometry is constrained to fit within a 180x180mm print bed, necessitating either a modular interlocking frame design or a compact central hub with detachable arms to accommodate the 5-inch prop clearance.
* **Mounting:** Integrated standoffs for the ESP32, MPU6500, and 30x30mm/20x20mm ESC mounting patterns. 

## Development Roadmap

- [x] **Phase 1: Component Bench Testing**
  - Verify MPU6500 I2C/SPI communication with ESP32.
  - Calibrate ESC and test PWM motor driving at 6S voltages.
- [ ] **Phase 2: Mechanical Assembly**
  - Finalize SolidWorks assembly and print initial chassis prototype.
  - Mount electronics and isolate power delivery systems.
- [ ] **Phase 3: Flight Stack Development**
  - Implement and tune C++ PID loops.
  - Test basic autonomous guidance tasks (tethered hover test).
- [ ] **Phase 4: Telemetry & Control**
  - Integrate custom RC interface for pilot-in-the-loop control.
  - Expand flight envelope from basic hovering to full 6-DOF manual flight.


