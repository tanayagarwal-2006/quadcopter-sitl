import socket
import struct
import numpy as np


class SITLBridge:

    def __init__(
        self,
        tx_ip="127.0.0.1",
        tx_port=5001,
        rx_port=5000
    ):

        self.tx_addr = (tx_ip,tx_port)

        self.tx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rx_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rx_socket.bind(("127.0.0.1", rx_port))

        self.rx_socket.settimeout(0.1)

    def send_packet(self,packet):
        self.tx_socket.sendto(packet,self.tx_addr)

    def receive_motors(self):
        try:
            data, _ = self.rx_socket.recvfrom(1024)

            return np.array(struct.unpack("<4f",data))
        
        except socket.timeout:
            return None