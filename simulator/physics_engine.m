function currentDroneState = physics_engine(motor_speeds,prevState,params)
%Constants
g=9.81;
dt=0.001;

%Simulation and model parameters
mass=params.mass;
inertia_matrix=params.im;
arm_length=params.length;
kThr=params.kThr;
kTor=params.KTor;

%Previous state
position=prevState.pos(:);
angular_rate=prevState.omegas(:);
q=prevState.q(:);
velocity=prevState.vel(:);

%Thrust and Torque
T=kThr*motor_speeds.^2;
tau_motor=kTor*motor_speeds.^2;

%Forces
Fg=[0;0;mass*g];
F_thrust_body=[0;0;-sum(T)];

q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);

Rotation_matrix=[
2*(q0^2 + q1^2) - 1,     2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2);
2*(q1*q2 + q0*q3),       2*(q0^2 + q2^2) - 1,   2*(q2*q3 - q0*q1);
2*(q1*q3 - q0*q2),       2*(q2*q3 + q0*q1),     2*(q0^2 + q3^2) - 1  
];

F_thrust_world=Rotation_matrix*F_thrust_body;
F_total=Fg+F_thrust_world;

accel = F_total/mass;

% Motor-1: Front right, Motor-2: Back left, Motor-3: Front left, Motor-3: Back right
%Roll, Pitch, Yaw
drone_torques = ...
[
(arm_length/1.414213562)*((T(2)+T(3)) - (T(1)+T(4)));
(arm_length/1.414213562)*((T(2)+T(4)) - (T(1)+T(3)));
tau_motor(1) + tau_motor(2) - tau_motor(3) - tau_motor(4)
];

omega_changing = inertia_matrix\(drone_torques - ...
    cross(angular_rate,inertia_matrix*angular_rate));

%Updating drone state
velocity = velocity + accel*dt;
angular_rate = angular_rate + omega_changing*dt;
position = position + velocity*dt;

rate_matrix = [
    0 , -angular_rate(1) , -angular_rate(2) , -angular_rate(3);
    angular_rate(1) , 0 , angular_rate(3) , -angular_rate(2);
    angular_rate(2) , -angular_rate(3) , 0 , angular_rate(1);
    angular_rate(3) , angular_rate(2) , -angular_rate(1) , 0

];

q_changing = 0.5 * rate_matrix * q;
q = q + q_changing*dt;
q=q/norm(q);

prevState.pos=position;
prevState.omegas=angular_rate;
prevState.vel=velocity;
prevState.q=q;

currentDroneState=prevState;
end