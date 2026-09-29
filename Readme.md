# Ottolink

### Design and Control of a 4-DoF Anthropomorphic Robotic Manipulator

Ottolink is a custom 4-DoF anthropomorphic robotic manipulator designed and built for spatial end-effector trajectory execution. The project combines mechanical design, embedded real-time control, Jacobian-based inverse kinematics, custom electronics, and MuJoCo simulation into a complete robotic system.

![Ottolink](Static/OttolinkLogo.png)

---

## Overview

Ottolink features a **3-DoF shoulder configuration and 1-DoF pitch elbow**, providing spatial control of the end effector.

The system was designed with a focus on implementing the complete robotics stack:

**CAD → Mechanical Assembly → Kinematics → Simulation → Embedded Control → Custom PCB → Physical Robot**

The controller runs on an **ATmega328P-based embedded system** and executes Cartesian-space trajectories using a Jacobian pseudoinverse controller.

---

## Features

- 4-DoF anthropomorphic robotic arm
- 3-DoF shoulder + 1-DoF pitch elbow
- Custom mechanical design in Autodesk Inventor
- 3D-printable mechanical components
- Jacobian pseudoinverse inverse kinematics
- Real-time Cartesian-space control
- Custom C++ linear algebra library
- Rodrigues rotation formulation for joint-axis transformations
- Multi-axis synchronized PWM servo control
- Joint interpolation and trajectory execution
- Custom control PCB designed in KiCad
- Dedicated power distribution and voltage regulation
- MuJoCo simulation environment
- Simulation-to-hardware trajectory validation

---

## System Architecture

```text
                   Cartesian Target
                         │
                         ▼
                ┌─────────────────┐
                │ Cartesian       │
                │ Controller      │
                └────────┬────────┘
                         │
                         ▼
                ┌─────────────────┐
                │ Jacobian        │
                │ Pseudoinverse   │
                └────────┬────────┘
                         │
                         ▼
                ┌─────────────────┐
                │ Joint-Space     │
                │ Trajectory      │
                └────────┬────────┘
                         │
                         ▼
                ┌─────────────────┐
                │ ATmega328P      │
                │ Controller      │
                └────────┬────────┘
                         │
                         ▼
                ┌─────────────────┐
                │ Multi-Axis PWM  │
                │ Servo Control   │
                └────────┬────────┘
                         │
                         ▼
                  Physical Robot