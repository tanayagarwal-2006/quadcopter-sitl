#include <iostream>
#include <iomanip>
#include <cmath>
#include "vector_axes.h"
#include "angle_controller.h"
#include "pid_rate_controller.h"
#include "motor_mixer.h"

int main(){
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
    
    vector3 pilot_setpoint = {0.0f, 0.0f, 1.0f};
    float base_hover_throttle = 0.5f; 

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Time\t| Yaw Rate\t| Trq Cmd(Z)\t| w1(CCW)\t| w2(CCW)\t| w3(CW) \t| w4(CW)\n";
    std::cout << "----------------------------------------------------------------------------------------------------\n";

    float MAX_RADS = 1000.0f; 
    float K_torque = 1.0f / (4.0f * MAX_RADS * MAX_RADS);
    
    float K_yaw_torque = K_torque * 0.1f; 

    for (int i = 0; i <= 1000; ++i) { 
        float time = i * dt;

        vector3 rate_cmd = angle_ctrl.update(pilot_setpoint, current_angle);
        vector3 torque_cmd = rate_ctrl.update(rate_cmd, current_rate);
        motor_mixer::motor_commands motors = mixer.mix(base_hover_throttle, torque_cmd);

        float w1 = MAX_RADS * std::sqrt(motors.m1);
        float w2 = MAX_RADS * std::sqrt(motors.m2);
        float w3 = MAX_RADS * std::sqrt(motors.m3);
        float w4 = MAX_RADS * std::sqrt(motors.m4);

        float generated_yaw_torque = K_yaw_torque * ((w1*w1 + w2*w2) - (w3*w3 + w4*w4));

        float angular_accel = generated_yaw_torque / inertia;
        current_rate.z += angular_accel * dt;

        current_angle.z += current_rate.z * dt; 

        if (i % 10 == 0) {
            std::cout << time << "\t| " 
                      << current_rate.z << "\t\t| " 
                      << torque_cmd.z << "\t\t| " 
                      << w1 << "\t| " << w2 << "\t| " << w3 << "\t| " << w4 << "\n";
        }
    }
    return 0;
}