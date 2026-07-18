import struct

def build_controller_packet(gyro,accel,vel,true_yaw_angle):
    return struct.pack(
        "<10f",
        gyro[0],
        gyro[1],
        gyro[2],

        accel[0],
        accel[1],
        accel[2],

        vel[0],
        vel[1],
        vel[2],

        true_yaw_angle
    )