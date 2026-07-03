# Quadcopter SITL
A custom Software-in-the-Loop (SITL) quadcopter platform built from scratch to develop a first-principles understanding of quadrotor flight dynamics, 
control systems, and state estimation.
The project couples a Python-based 6-DOF flight dynamics simulator with a C++ flight controller through a high-frequency UDP interface.

## Current Features

### Flight Dynamics
- 6-DOF rigid-body dynamics using Newton-Euler equations
- Quaternion-based attitude representation and propagation
- Rotor thrust and reaction torque generation
- First-order motor response dynamics
- Linear aerodynamic drag modelling
  
### Flight Controller
- Cascaded PID attitude and angular-rate controller
- Analytical control allocation for X-configuration quadrotors
- Saturation-aware anti-windup protection
- Controller output allocation to individual motors

### Sensor Simulation
- Stochastic IMU model
- Accelerometer and gyroscope measurement noise
- Sensor bias drift simulation

### State Estimation
- Complementary-filter-based attitude estimation

### Communication
- High-frequency UDP communication between the simulator and controller

### Tooling
- Telemetry logging
- Graph-based visualization


## Repository Structure
```
quadcopter-sitl/
│
├── controller/          C++ flight controller
├── simulator/           6-DOF flight dynamics
├── imu/                 IMU sensor model
├── estimation/          Attitude estimation
├── communication/       UDP interface
├── telemetry/           Logging and visualization
├── tests/               Test utilities
```

## Current Status
The project is under active development.
