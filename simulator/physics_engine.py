import numpy as np

def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,

        w1*x2 + x1*w2 + y1*z2 - z1*y2,

        w1*y2 - x1*z2 + y1*w2 + z1*x2,

        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def current_drone_state(motor_speeds, prev_state, params):
    '''
    motor_speeds: array
    prev_state: dict
    params: dict
    '''

    #Constants
    g = 9.81
    dt = params["dt"]

    #parameters
    mass=params["mass"]
    inertia_matrix=params["inertia_matrix"]
    arm_length=params["arm_length"]
    k_thrust=params["k_thrust"]
    k_torque=params["k_torque"]
    max_motor_speed=params["max_motor_speed"]

    position = prev_state["pos"].copy()
    angular_rate = prev_state["omegas"].copy()
    q = prev_state["q"].copy()
    velocity = prev_state["vel"].copy()

    #Thrust and Torque
    motor_actual = prev_state["motor_actual"].copy()

    motor_actual += (
        (motor_speeds - motor_actual)
        * dt / params["tau_motor"]
    )

    motor_actual = np.clip(
        motor_actual,
        0.0,
        max_motor_speed
    )

    thrust = k_thrust * (motor_actual**2)
    motor_torque = k_torque * (motor_actual**2)

    #Forces
    F_ground=np.array([0.0,0.0,mass*g])
    F_thrust_body=np.array([0.0, 0.0, -sum(thrust)])

    q0, q1, q2, q3 = q

    rotation_matrix = np.array([
    [
        2*(q0**2+q1**2)-1, 
        2*(q1*q2-q0*q3), 
        2*(q1*q3+q0*q2)
    ],
    [
        2*(q1*q2+q0*q3),
        2*(q0**2+q2**2)-1,
        2*(q2*q3-q0*q1)
    ],
    [
        2*(q1*q3-q0*q2),
        2*(q2*q3+q0*q1),
        2*(q0**2+q3**2)-1
    ]
    ])

    F_thrust_world=rotation_matrix@F_thrust_body
    F_total=F_ground+F_thrust_world

    acceleration=F_total/mass

    # Motor-1: Front right, Motor-2: Back left, 
    # Motor-3: Front left Motor-4: Back right
    # Roll, Pitch, Yaw
    drone_torques = np.array([
        (arm_length / np.sqrt(2.0))
        * ((thrust[1] + thrust[2]) - (thrust[0] + thrust[3])),

        (arm_length / np.sqrt(2.0))
        * ((thrust[1] + thrust[3]) - (thrust[0] + thrust[2])),

        motor_torque[0]
        + motor_torque[1]
        - motor_torque[2]
        - motor_torque[3]
    ])

    omega_changing=np.linalg.solve(
        inertia_matrix,
        drone_torques-
        np.cross(
            angular_rate,
            inertia_matrix @ angular_rate
        )
    )

    velocity = velocity + acceleration * dt
    angular_rate = angular_rate + omega_changing * dt
    position = position + velocity * dt

    if position[2]>0.0:
        position[2] = 0.0
        velocity[2] = 0.0

    '''roll_rate, pitch_rate, yaw_rate=angular_rate
    rate_matrix=np.array(
        [
            [0, -roll_rate, -pitch_rate, -yaw_rate],
        [roll_rate, 0, yaw_rate, -pitch_rate],
        [pitch_rate, -yaw_rate, 0, roll_rate],
        [yaw_rate, pitch_rate, -roll_rate, 0]
        ]
    )

    q_changing=0.5*(rate_matrix@q)
    q+=q_changing*dt
    q/=np.linalg.norm(q)'''

    omega_norm = np.linalg.norm(angular_rate)

    if omega_norm > 1e-8:

        theta = omega_norm * dt

        axis = angular_rate / omega_norm

        dq = np.array([
            np.cos(theta / 2.0),

            axis[0] * np.sin(theta / 2.0),

            axis[1] * np.sin(theta / 2.0),

            axis[2] * np.sin(theta / 2.0)
        ])

    else:
        dq = np.array([1.0, 0.0, 0.0, 0.0])

    q = quat_multiply(q, dq)

    q /= np.linalg.norm(q)

    current_drone_state={
        "pos": position,
        "omegas": angular_rate,
        "vel": velocity,
        "q": q,
        "acc_world": acceleration,
        "motor_actual": motor_actual
    }

    return current_drone_state