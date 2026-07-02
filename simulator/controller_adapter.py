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
    return np.array([roll,pitch,yaw])

def build_euler_controller_packet(truth_state):
    return {
        "euler_rad": quat_to_euler(truth_state["quat"]),
        "body_rates": truth_state["body_rates"]
    }