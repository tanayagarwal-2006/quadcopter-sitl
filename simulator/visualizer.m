clear; clc; close all;

import java.net.DatagramSocket
import java.net.DatagramPacket

try
    udp_rx = DatagramSocket(5002);
    udp_rx.setSoTimeout(10); 
catch
    error('Port 5002 is locked. Restart MATLAB.');
end
cleanup_rx = onCleanup(@() udp_rx.close());

% Setup 3D Scene
fig = figure('Name', 'Drone Visualizer', 'NumberTitle', 'off');
ax = axes('Parent', fig);
view(3); grid on; hold on;

% Expanded boundaries to watch the drift
axis([-300 300 -300 300 -100 100]); 
xlabel('X (North)'); ylabel('Y (East)'); zlabel('Z (Down is Positive)');

%+Z point downwards
set(ax, 'ZDir', 'reverse'); 

% add trail
trail = animatedline('Color', 'g', 'LineWidth', 1.5, 'MaximumNumPoints', 10000);

drone_transform = hgtransform('Parent', ax);
L = 0.25;
line([-L L], [-L L], [0 0], 'Color', 'r', 'LineWidth', 3, 'Parent', drone_transform);
line([-L L], [L -L], [0 0], 'Color', 'b', 'LineWidth', 3, 'Parent', drone_transform);
scatter3(L, L, 0, 100, 'r', 'filled', 'Parent', drone_transform); 

disp('Visualizer Live. Waiting for telemetry...');

buffer = zeros(1, 24, 'int8');
packet = DatagramPacket(buffer, 24);

while true
    try
        udp_rx.receive(packet);
        data = packet.getData();
        floats = typecast(data, 'single');
        
        x = floats(1); y = floats(2); z = floats(3);
        roll = floats(4); pitch = floats(5); yaw = floats(6);
        
        T = makehgtform('translate', x, y, z);
        R = makehgtform('xrotate', roll, 'yrotate', pitch, 'zrotate', yaw);
        drone_transform.Matrix = T * R;
        
        % UPDATE THE TRAIL
        addpoints(trail, x, y, z);
        
        drawnow limitrate;
    catch
        pause(0.001);
        drawnow limitrate;
    end
end