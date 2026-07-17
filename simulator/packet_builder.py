import struct

def build_euler_packet(euler_rad,body_rates,vel,true_yaw_angle):
    return struct.pack(
        "<10f",
        euler_rad[0],
        euler_rad[1],
        euler_rad[2],

        body_rates[0],
        body_rates[1],
        body_rates[2],

        vel[0],
        vel[1],
        vel[2],

        true_yaw_angle
    )