import numpy as np

def quat_to_euler(q):
    qw, qx, qy, qz = q
    roll = np.arctan2(
        2.0 * (qw*qx + qy*qz),
        1.0 - 2.0 * (qx*qx + qy*qy)
    )
    pitch = np.arcsin(
        np.clip(
            2.0 * (qw*qy - qz*qx),
            -1.0,
            1.0
        )
    )
    yaw = np.arctan2(
        2.0 * (qw*qz + qx*qy),
        1.0 - 2.0 * (qy*qy + qz*qz)
    )
    return np.array([
        roll,
        pitch,
        yaw
    ])

class FlightLogger():
    def __init__(self):
        self.time = []

        self.x_deg_phi=[]
        self.y_deg_theta=[]
        self.z_deg_psi=[]

        self.roll_rate = []
        self.pitch_rate = []
        self.yaw_rate = []

        self.x = []
        self.y = []
        self.z = []

        self.vx = []
        self.vy = []
        self.vz = []

        self.ax = []
        self.ay = []
        self.az = []

        self.m1 = []
        self.m2 = []
        self.m3 = []
        self.m4 = []

    def log(self, t, state, motor_commands):
        self.time.append(t)
        euler_rad = quat_to_euler(state["quat"])
        self.x_deg_phi.append(np.degrees(euler_rad[0]))
        self.y_deg_theta.append(np.degrees(euler_rad[1]))
        self.z_deg_psi.append(np.degrees(euler_rad[2]))
        r,p,y = np.degrees(state["body_rates"])
        self.roll_rate.append(r)
        self.pitch_rate.append(p)
        self.yaw_rate.append(y)
        self.x.append(state["pos"][0])
        self.y.append(state["pos"][1])
        self.z.append(state["pos"][2])
        self.vx.append(state["vel"][0])
        self.vy.append(state["vel"][1])
        self.vz.append(state["vel"][2])
        self.ax.append(state["acc_world"][0])
        self.ay.append(state["acc_world"][1])
        self.az.append(state["acc_world"][2])
        self.m1.append(motor_commands[0])
        self.m2.append(motor_commands[1])
        self.m3.append(motor_commands[2])
        self.m4.append(motor_commands[3])