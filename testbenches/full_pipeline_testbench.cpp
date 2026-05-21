#include "udp_bridge.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <windows.h>
#include "vector_axes.h"
#include "angle_controller.h"
#include "pid_rate_controller.h"
#include "motor_mixer.h"
#include "utils.h"


int main() {
    float dt = 0.001f; 

    udp_bridge matlab_link("127.0.0.1", 5000, 5001);
    std::cout << "C++ Brain Online (1000Hz). Waiting for MATLAB SITL...\n";

    vector2 angle_kp = {2.0f, 2.0f}; 
    vector3 max_rates = {6.28f, 6.28f, 2.0f}; 
    angle_controller angle_ctrl(angle_kp, max_rates, 45.0f);

    PID_parameters roll_pid = {1.4f, 0.2f, 0.05f, 10.0f}; 
    PID_parameters pitch_pid = {1.4f, 0.2f, 0.05f, 10.0f};
    PID_parameters yaw_pid = {1.0f, 0.2f, 0.00f, 10.0f};
    rate_controller rate_ctrl(roll_pid, pitch_pid, yaw_pid, dt);

    motor_mixer mixer(0.0f, 1.0f); 

    state_payload current_state = {{0,0,0}, {0,0,0}};
    float MAX_RADS = 1000.0f; 
    float base_hover_throttle = 0.5f; 
    
    long long tick_counter = 0;

    while (true) {
        float current_time = tick_counter * dt;

        vector3 pilot_setpoint = {0.0f, 0.0f, 0.0f};
        if (current_time > 2.0f && current_time <= 3.0f) {
            pilot_setpoint.y = 30.0f * 3.14159f / 180.0f; 
        } else if (current_time > 4.0f && current_time <= 5.0f) {
            pilot_setpoint.x = 30.0f * 3.14159f / 180.0f; 
        }

        matlab_link.receive_state(current_state);

        vector3 rate_cmd = angle_ctrl.update(pilot_setpoint, current_state.current_angle);
        vector3 torque_cmd = rate_ctrl.update(rate_cmd, current_state.current_rate);

        //FEED-FORWARD THROTTLE COMPENSATION
        // Extract current angles (assuming they are in radians from your simulation)
        float current_roll = current_state.current_angle.x;
        float current_pitch = current_state.current_angle.y;

        float cos_roll = std::cos(current_roll);
        float cos_pitch = std::cos(current_pitch);

        // Safety clamp: Prevent division by zero or massive throttle spikes 
        // if the drone hits an extreme angle (limits compensation to ~84 degrees of tilt)
        if (cos_roll < 0.1f) cos_roll = 0.1f;
        if (cos_pitch < 0.1f) cos_pitch = 0.1f;

        // Calculate the boosted throttle
        float dynamic_throttle = base_hover_throttle / (cos_roll * cos_pitch);

        motor_mixer::motor_commands motors = mixer.mix(dynamic_throttle, torque_cmd);

        motor_payload out_motors;
        out_motors.m1_rads = MAX_RADS * std::sqrt(motors.m1);
        out_motors.m2_rads = MAX_RADS * std::sqrt(motors.m2);
        out_motors.m3_rads = MAX_RADS * std::sqrt(motors.m3);
        out_motors.m4_rads = MAX_RADS * std::sqrt(motors.m4);
        matlab_link.send_motors(out_motors);

        if (tick_counter % 100 == 0) {
            std::cout << "Time: " << std::fixed << std::setprecision(1) << current_time 
                      << "s | R: " << std::setprecision(2) << current_state.current_angle.x * 180.0f / 3.14159f
                      << " | P: " << current_state.current_angle.y * 180.0f / 3.14159f
                      << " | Y: " << current_state.current_angle.z * 180.0f / 3.14159f << "\n";
        }

        tick_counter++;
        Sleep(1);
    }
    return 0;
}