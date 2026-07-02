#ifndef UDP_BRIDGE_H
#define UDP_BRIDGE_H

#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#endif

#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include "vector_axes.h"

#pragma comment(lib, "ws2_32.lib")

struct motor_payload {
    float m1_rads;
    float m2_rads;
    float m3_rads;
    float m4_rads;
};

// Packet received FROM simulator (24 bytes)
/*struct state_payload {
    vector3 current_angle; 
    vector3 current_rate;  
    
};*/

struct state_payload {
    vector3 gyroscope; 
    vector3 accelerometer; 
    float yaw_angle;  
};

class udp_bridge {
    private:
        SOCKET send_socket;
        SOCKET recv_socket;
        sockaddr_in simulation_addr;
        sockaddr_in local_addr;

    public:
        // Initialize sockets (Target IP, Send Port, Listen Port)
        udp_bridge(const std::string& target_ip, int send_port, int recv_port);
        ~udp_bridge();

        void send_motors(const motor_payload& motors);
        
        bool receive_state(state_payload& state);
};

#endif