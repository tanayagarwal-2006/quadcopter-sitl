#include "udp_bridge.h"
#include <iostream>

udp_bridge::udp_bridge(const std::string& target_ip, int send_port, int recv_port) {
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        std::cerr << "WSAStartup failed.\n";
        exit(1);
    }   

    // Setup Send Socket
    send_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (send_socket == INVALID_SOCKET) {
        std::cerr << "Failed to create send socket.\n";
        WSACleanup();
        exit(1);
    }
    simulation_addr.sin_family = AF_INET;
    simulation_addr.sin_port = htons(send_port);
    inet_pton(AF_INET, target_ip.c_str(), &simulation_addr.sin_addr);

    // Setup Receive Socket
    recv_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (recv_socket == INVALID_SOCKET) {
        std::cerr << "Failed to create receive socket.\n";
        WSACleanup();
        exit(1);
    }
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(recv_port);
    local_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(recv_socket, (sockaddr*)&local_addr, sizeof(local_addr)) == SOCKET_ERROR) {
        std::cerr << "Bind failed: " << WSAGetLastError() << "\n";
        closesocket(recv_socket);
        WSACleanup();
        exit(1);
    }

    //u_long mode = 1; 
    //ioctlsocket(recv_socket, FIONBIO, &mode);
}

udp_bridge::~udp_bridge() {
    closesocket(send_socket);
    closesocket(recv_socket);
    WSACleanup();
}

void udp_bridge::send_motors(const motor_payload& motors) {
    sendto(send_socket, (const char*)&motors, sizeof(motor_payload), 0, 
           (sockaddr*)&simulation_addr, sizeof(simulation_addr));
}

bool udp_bridge::receive_state(state_payload& state) {
    sockaddr_in sender_addr;
    int sender_addr_size = sizeof(sender_addr);
    
    int bytesIn = recvfrom(recv_socket, (char*)&state, sizeof(state_payload), 0, 
                           (sockaddr*)&sender_addr, &sender_addr_size);

    if (bytesIn == SOCKET_ERROR) {
        int error = WSAGetLastError();
        if (error == WSAEWOULDBLOCK) {
            return false;
        }
        std::cerr << "Receive error: " << error << "\n";
        return false;
    }

    return (bytesIn == sizeof(state_payload));
}