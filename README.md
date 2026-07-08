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

```
quadcopter-sitl/
│
├── simulator/
│   ├── physics_engine.py      # 6-DOF quadrotor dynamics
│   ├── imu_model.py           # IMU sensor simulation
│   ├── controller_adapter.py  
│   ├── udp_bridge.py          # UDP communication
│   ├── packet_builder.py      
│   ├── config.py              # Simulation parameters
│   ├── main.py                # Main simulation loop
│   └── visualizer/
│       ├── logger.py
│       └── plotter.py
│
├── controller/
│   ├── angle_controller.*     # Outer-loop attitude controller
│   ├── pid_rate_controller.*  # Inner-loop angular-rate PID
│   ├── motor_mixer.*          # Control allocation
│   ├── state_estimator.*      # Attitude estimation
│   ├── udp_bridge.*           # Simulator interface
│   └── utils.*
│
└── testbenches/
    └── full_pipeline_testbench.cpp
    └── deprecated_tests/
        ├── disturbance_testbench.cpp
        └── pitch_testbench.cpp
        ├── roll_testbench.cpp
        └── wind_testbench.cpp
        └── yaw_testbench.cpp
        
```

## Current Status
The project is under active development.
