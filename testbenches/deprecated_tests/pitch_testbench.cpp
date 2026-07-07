#include <iostream>
#include <iomanip>
#include <cmath>
#include "vector_axes.h"
#include "angle_controller.h"
#include "pid_rate_controller.h"
#include "motor_mixer.h"

int main() {
    float dt = 0.01f; 
    float inertia = 0.05f; 

    vector2 angle_kp = {2.0f, 2.0f}; 
    vector3 max_rates = {6.28f, 6.28f, 2.0f}; 
    angle_controller angle_ctrl(angle_kp, max_rates, 45.0f);

    PID_parameters roll_pid = {1.4f, 0.2f, 0.05f, 10.0f}; 
    PID_parameters pitch_pid = {1.4f, 0.2f, 0.05f, 10.0f};
    PID_parameters yaw_pid = {1.0f, 0.2f, 0.00f, 10.0f}; 
    rate_controller rate_ctrl(roll_pid, pitch_pid, yaw_pid, dt);

    motor_mixer mixer(0.0f, 1.0f); 

    vector3 current_angle = {0.0f, 0.0f, 0.0f};
    vector3 current_rate = {0.0f, 0.0f, 0.0f};
    
    vector3 pilot_setpoint = {0.0f, 45.0f * 3.14159f / 180.0f, 0.0f};
    float base_hover_throttle = 0.5f; 

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time\t| Pitch(deg)\t| Rate(rad/s)\t| Trq Cmd\t| w1(Front)\t| w2(Back)\t| w3(Front)\t| w4(Back)\n";
    std::cout << "------------------------------------------------------------------------------------------------------------\n";

    float MAX_RADS = 1000.0f; 
    float K_pitch_torque = 5.0f / (MAX_RADS * MAX_RADS); 

    for (int i = 0; i <= 300; ++i) { 
        float time = i * dt;

        vector3 rate_cmd = angle_ctrl.update(pilot_setpoint, current_angle);
        vector3 torque_cmd = rate_ctrl.update(rate_cmd, current_rate);
        motor_mixer::motor_commands motors = mixer.mix(base_hover_throttle, torque_cmd);

        float w1 = MAX_RADS * std::sqrt(motors.m1);
        float w2 = MAX_RADS * std::sqrt(motors.m2);
        float w3 = MAX_RADS * std::sqrt(motors.m3);
        float w4 = MAX_RADS * std::sqrt(motors.m4);

        float generated_pitch_torque = K_pitch_torque * ((w2*w2 + w4*w4) - (w1*w1 + w3*w3));

        float angular_accel = generated_pitch_torque / inertia;
        current_rate.y += angular_accel * dt;
        current_angle.y += current_rate.y * dt; 

        if (i % 10 == 0) {
            std::cout << time << "\t| " 
                      << current_angle.y * 180.0f / 3.14159f << "\t\t| " 
                      << current_rate.y << "\t\t| " 
                      << torque_cmd.y << "\t\t| " 
                      << w1 << "\t| " << w2 << "\t| " << w3 << "\t| " << w4 << "\n";
        }
    }
    return 0;
}