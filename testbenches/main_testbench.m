clc;
clear;
close all;

%% ================= PARAMETERS =================
params.mass   = 0.1;
params.length = 0.25;
params.kThr   = 1.8e-6;
params.KTor   = 2.5e-7;
params.im     = diag([0.02 0.02 0.04]);

dt = 0.001;        % 1 kHz physics

%% ================= INITIAL STATE =================
prevState.pos    = [0;0;0];
prevState.vel    = [0;0;0];
prevState.omegas = [0;0;0];
prevState.q      = [1;0;0;0];

%% ================= FIGURE =================
figure('Color','w');
hold on;
grid on;
view(3);

axis equal;
daspect([1 1 1]);

axis_lim = 1000;
xlim([-axis_lim axis_lim]);
ylim([-axis_lim axis_lim]);
zlim([-axis_lim axis_lim]);

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Open Loop Roll Torque Test');

traj   = plot3(0,0,0,'b','LineWidth',2);
marker = plot3(0,0,0,'ro','MarkerFaceColor','r','MarkerSize',10);

pos_log = [];
t = 0;
k = 0;

%% ================= MAIN LOOP =================
while true
    
    k = k + 1;
    
    % ===== Constant Roll Torque =====
    motor_speeds = [
        4000;   % Motor 1
        6000;   % Motor 2
        4000;   % Motor 3
        6000    % Motor 4
    ];
    
    currentDroneState = physics_engine(motor_speeds, prevState, params);
    prevState = currentDroneState;
    
    % Convert NED → Up
    pos_plot = currentDroneState.pos(:);
    pos_plot(3) = -pos_plot(3);
    
    pos_log = [pos_log pos_plot];
    
    % 500 Hz visualization
    if mod(k,2) == 0
        set(traj,'XData',pos_log(1,:), ...
                 'YData',pos_log(2,:), ...
                 'ZData',pos_log(3,:));
             
        set(marker,'XData',pos_plot(1), ...
                   'YData',pos_plot(2), ...
                   'ZData',pos_plot(3));
        
        drawnow;
    end
    
    pause(dt);
    t = t + dt;

    if mod(k,100) == 0   % Print at 100 Hz (1 kHz / 10)
        % ================= PRINT STATES =================
    fprintf('\nTime: %.4f s\n', t);
    
    fprintf('Position (m):      X = %+8.4f   Y = %+8.4f   Z = %+8.4f\n', ...
        currentDroneState.pos(1), ...
        currentDroneState.pos(2), ...
        currentDroneState.pos(3));
    
    fprintf('Velocity (m/s):    VX = %+8.4f  VY = %+8.4f  VZ = %+8.4f\n', ...
        currentDroneState.vel(1), ...
        currentDroneState.vel(2), ...
        currentDroneState.vel(3));
    
    fprintf('Angular Rates (rad/s):  p = %+8.4f  q = %+8.4f  r = %+8.4f\n', ...
        currentDroneState.omegas(1), ...
        currentDroneState.omegas(2), ...
        currentDroneState.omegas(3));
    
    fprintf('Quaternion:        w = %+8.4f  x = %+8.4f  y = %+8.4f  z = %+8.4f\n', ...
        currentDroneState.q(1), ...
        currentDroneState.q(2), ...
        currentDroneState.q(3), ...
        currentDroneState.q(4));
    end
end
