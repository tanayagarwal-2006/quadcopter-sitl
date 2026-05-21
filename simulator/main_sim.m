clear; clc;

% 1. Import Native Java Networking
import java.net.DatagramSocket
import java.net.DatagramPacket
import java.net.InetAddress

% 2. Setup Java Sockets
try
    udp_rx = DatagramSocket(5000);
    udp_rx.setSoTimeout(1);
catch
    error('Port 5000 locked. Type "clear all" or restart MATLAB.');
end

% Outbound Sockets (No local port bound)
udp_tx = DatagramSocket(); 
addr_target = InetAddress.getByName('127.0.0.1');

% Failsafe Socket Cleanup
cleanup_rx = onCleanup(@() udp_rx.close());
cleanup_tx = onCleanup(@() udp_tx.close());

disp('MATLAB SITL Live (Java Sockets). Waiting for C++...');

% 3. Drone Physical Parameters
params.mass = 1.0; 
params.im = diag([0.05, 0.05, 0.10]); 
params.length = 0.25; 
MAX_RADS = 1000.0;
params.kThr = 9.81 / (2.0 * MAX_RADS^2); 
params.KTor = params.kThr * 0.1; 
params.dt = 0.001; 

prevState.pos = [0; 0; 10]; 
prevState.omegas = [0; 0; 0];   
prevState.vel = [0; 0; 0];
prevState.q = [1; 0; 0; 0]; 

% 4. Preallocate Incoming Buffer
rx_buffer = zeros(1, 16, 'int8');
rx_packet = DatagramPacket(rx_buffer, 16);

while true
    try
        % Attempt to read packet
        udp_rx.receive(rx_packet);
        
        % Convert bytes to floats
        data = rx_packet.getData();
        motor_speeds = double(typecast(data, 'single'))'; 
        
        % Step physics
        currentState = physics_engine(motor_speeds, prevState, params);
        
        q = currentState.q;
        roll  = atan2(2*(q(1)*q(2) + q(3)*q(4)), 1 - 2*(q(2)^2 + q(3)^2));
        pitch = asin(2*(q(1)*q(3) - q(4)*q(2)));
        yaw   = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)^2 + q(4)^2));
        
        % SEND TO C++ (Port 5001) -
        cpp_payload = single([roll, pitch, yaw, currentState.omegas(1), currentState.omegas(2), currentState.omegas(3)]);
        cpp_bytes = typecast(cpp_payload, 'int8');
        tx_packet_cpp = DatagramPacket(cpp_bytes, length(cpp_bytes), addr_target, 5001);
        udp_tx.send(tx_packet_cpp);
        
        % SEND TO VISUALIZER (Port 5002) -
        vis_payload = single([currentState.pos(1), currentState.pos(2), currentState.pos(3), roll, pitch, yaw]);
        vis_bytes = typecast(vis_payload, 'int8');
        tx_packet_vis = DatagramPacket(vis_bytes, length(vis_bytes), addr_target, 5002);
        udp_tx.send(tx_packet_vis);
        
        prevState = currentState;
    catch
        % Timeout, loop again
    end
end