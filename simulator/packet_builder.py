import struct

def build_euler_packet(angles,rates):
    return struct.pack(
        "<6f",
        angles[0],
        angles[1],
        angles[2],

        rates[0],
        rates[1],
        rates[2]
    )