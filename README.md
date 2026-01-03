# BLDC Motor Speed Control Simulation (Python)

## Overview
This repository contains a **physics-based Brushless DC (BLDC) motor simulation** implemented in Python.
The model captures the **electrical, mechanical, inverter, commutation, and control behavior** of a three-phase BLDC motor using **120° trapezoidal back-EMF and six-step commutation**, along with a **closed-loop PI speed controller**.

The objective is to provide a **clear, extensible, and engineering-oriented reference model** that can be used for learning, validation, and early-stage motor control algorithm development.

---

## Objectives
The main goals of this project are:

- Model BLDC **electrical dynamics** (phase currents, back-EMF, inverter voltages)
- Implement **six-step (120°) commutation logic**
- Represent **trapezoidal back-EMF waveforms**
- Develop a **PI-based speed control loop**
- Simulate **electromagnetic torque production**
- Capture **mechanical rotor dynamics**
- Provide a **clean baseline** for extending into advanced motor control algorithms

---

## Features
- Trapezoidal back-EMF BLDC motor model
- Six-step inverter commutation (120° conduction)
- Floating neutral (star-connected windings, no neutral wire)
- Phase current dynamics using RL differential equations
- PI speed controller with saturation and anti-windup
- Mechanical dynamics with inertia and viscous friction
- Electrical angle wrapping for numerical stability
- Time-domain visualization of all key signals

---

## File Structure
```
.
├── BLDC_motor_with_speed_controller.py
└── README.md
```

This project is intentionally kept **single-file and self-contained** to make the control flow and physics easy to follow.

---

## File Description

### BLDC_motor_with_speed_controller.py

This script contains the complete BLDC motor simulation and is organized into logical sections:

1. **Motor Parameters**
   - Electrical parameters (R, L, Ke, Kt)
   - Mechanical parameters (J, friction)
   - DC bus voltage and pole pairs

2. **Speed Controller (PI)**
   - Closed-loop PI controller
   - Output saturation and anti-windup logic
   - Normalized duty-cycle command

3. **Inverter & Commutation**
   - Six-step commutation logic
   - Floating neutral voltage calculation
   - Phase voltage generation

4. **Electrical Dynamics**
   - RL phase current differential equations
   - Trapezoidal back-EMF modeling
   - Electromagnetic torque calculation

5. **Mechanical Dynamics**
   - Rotor acceleration and speed integration
   - Electrical angle generation and wrapping

6. **Simulation Loop**
   - Step-based speed reference profile
   - Signal logging for analysis

7. **Visualization**
   - Speed tracking
   - Phase voltages and currents
   - Torque and electrical angle
   - Back-EMF shape functions

---

## How to Run the Simulation

### Prerequisites
- Python 3.8 or newer
- Required Python package:
```bash
pip install matplotlib
```

### Run Command
```bash
python BLDC_motor_with_speed_controller.py
```

The script will execute the simulation and display multiple result plots automatically.

---

## How to Extend This Model

This simulation is designed as a **foundation for advanced motor control development**, including:

### Control Extensions
- Field-Oriented Control (FOC)
- Inner current control loops
- Torque control
- Regenerative braking

### System Enhancements
- Battery and DC-link dynamics
- PWM switching models
- Load torque profiles
- Thermal modeling

### Software Architecture
- Modular file separation (motor, inverter, controller)
- Higher-order numerical solvers (RK4)
- Logging and configuration files

---

## Intended Audience
- Motor Controls Engineers
- Embedded Software Engineers
- EV / Hybrid Powertrain Engineers
- Robotics and Mechatronics Engineers
- Students learning motor control fundamentals

---

## Disclaimer
This model is intended for **educational use and algorithm prototyping only**.
It is **not production-ready** and does not include hardware-specific non-idealities such as switching losses, sensor noise, or real-time constraints.

---

## 👤 Author
Developed by an Rajeshwar Yadav, Automotive Controls Engineer focusing on Model-Based Development, EV powertrain systems, and Python-based simulation.
Email ID : ryadav1@mtu.edu
LinkedIn: www.linkedin.com/in/rajeshwar-yadav-53710143
