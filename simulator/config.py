import numpy as np

# Frame: DJI450, Motors: SunnySky X2212 KV980, Battery: 4S(14.8V), Prop: APC9045
PARAMS = {
    "dt": 0.001,
    "mass": 1.2,
    "inertia_matrix": np.diag([
        0.0083,
        0.0083,
        0.0142
    ]),
    "arm_length": 0.225,
    "k_thrust": 7.34e-6,
    "k_torque": 9.80e-8,
    "max_motor_speed": 1150.0,
    "tau_motor": 0.04,
    "k_drag":0.28
}