#include "udp_bridge.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <windows.h>
#include "vector_axes.h"
#include "angle_controller.h"
#include "pid_rate_controller.h"
#include "motor_mixer.h"
#include "utils.h"

int main() {
    float dt = 0.001f; 

    udp_bridge simulation_link("127.0.0.1", 5000, 5001);
    std::cout << "C++ Brain Online (1000Hz). Waiting for SITL...\n";

    vector2 angle_kp = {1.0f, 1.0f}; 
    vector3 max_rates = {1.57f, 1.57f, 2.0f}; 
    angle_controller angle_ctrl(angle_kp, max_rates, 45.0f);

    /*PID_parameters roll_pid = {1.4f, 0.2f, 0.05f, 10.0f}; 
    PID_parameters pitch_pid = {1.4f, 0.2f, 0.05f, 10.0f};
    PID_parameters yaw_pid = {1.0f, 0.2f, 0.00f, 10.0f};*/

    /*PID_parameters roll_pid = {0.35f, 0.0f, 0.015f, 10.0f}; 
    PID_parameters pitch_pid = {0.35f, 0.0f, 0.015f, 10.0f};
    PID_parameters yaw_pid = {1.0f, 0.2f, 0.00f, 10.0f};*/

    PID_parameters roll_pid = {0.7f, 0.0f, 0.020f, 10.0f}; 
    PID_parameters pitch_pid = {0.7f, 0.0f, 0.020f, 10.0f};
    PID_parameters yaw_pid = {1.0f, 0.2f, 0.00f, 10.0f};

    rate_controller rate_ctrl(roll_pid, pitch_pid, yaw_pid, dt);

    motor_mixer mixer(0.0f, 1150*1150); 

    state_payload current_state = {{0,0,0}, {0,0,0}};
    float MAX_RADS = 1150.0f; 
    
    float hover_speed = sqrt((1.2f * 9.81f) / (4.0f * 7.34e-6f));
    //float base_hover_throttle = (hover_speed * hover_speed) / (1150.0f * 1150.0f);
    float base_hover_thrust = 1.2f * 9.81f;
    long long tick_counter = 0;

    auto interval = std::chrono::microseconds(1000); // Exactly 1000us (1ms)
    auto next_tick = std::chrono::steady_clock::now();

    // --- TABLE HEADER ---
    // Print this ONCE before entering the high-speed loop.
    std::cout << "\n===========================================================================================================\n";
    std::cout << std::left 
              << std::setw(9)  << "TIME(s)" 
              << std::setw(24) << " ANGLES: R/P/Y (deg)" 
              << std::setw(24) << " RATES: R/P/Y (rad/s)"
              << std::setw(22) << " TORQUE CMD (X/Y/Z)"
              << std::setw(10) << " DYN_THR"
              << std::setw(25) << " MOTORS: 1/2/3/4 (rad/s)" 
              << "\n";
    std::cout << "===========================================================================================================\n";

    while (true) {
        next_tick += interval;
        float current_time = tick_counter * dt;

        vector3 pilot_setpoint = {0.0f, 0.0f, 0.0f};
        /*if (current_time > 4.0f && current_time <= 7.0f) {
            pilot_setpoint.x = 30.0f * 3.14159f / 180.0f; 
        }*/

        /*if (current_time>=10.0f){
            break;
        }*/

        simulation_link.receive_state(current_state);

        vector3 rate_cmd = angle_ctrl.update(pilot_setpoint, current_state.current_angle);
        vector3 torque_cmd = rate_ctrl.update(rate_cmd, current_state.current_rate);

        // FEED-FORWARD THROTTLE COMPENSATION
        float current_roll = current_state.current_angle.x;
        float current_pitch = current_state.current_angle.y;

        /*float cos_roll = std::cos(current_roll);
        float cos_pitch = std::cos(current_pitch);*/
        float max_ff_angle = 0.698f;
        float eff_roll = constrain_to_value(std::abs(current_roll), 0.0f, max_ff_angle);
        float eff_pitch = constrain_to_value(std::abs(current_pitch), 0.0f, max_ff_angle);

        // Calculate thrust using the capped angles
        float dynamic_thrust_N = (base_hover_thrust / (std::cos(eff_roll) * std::cos(eff_pitch)))*1.1f;

        //float dynamic_throttle = base_hover_thrust / (cos_roll * cos_pitch);

        motor_mixer::motor_commands motors = mixer.mix(dynamic_thrust_N, torque_cmd);

        motor_payload out_motors;
        
        //float w1 = std::sqrt(constrain_to_value(motors.m1, 0.0f, MAX_RADS * MAX_RADS));
        //float w2 = std::sqrt(constrain_to_value(motors.m2, 0.0f, MAX_RADS * MAX_RADS));
        //float w3 = std::sqrt(constrain_to_value(motors.m3, 0.0f, MAX_RADS * MAX_RADS));
        //float w4 = std::sqrt(constrain_to_value(motors.m4, 0.0f, MAX_RADS * MAX_RADS));

        //out_motors.m1_rads = w1;
        //out_motors.m2_rads = w2;
        //out_motors.m3_rads = w3;
        //out_motors.m4_rads = w4;

        out_motors.m1_rads = 100;
        out_motors.m2_rads = 100;
        out_motors.m3_rads = 100;
        out_motors.m4_rads = 100;

        if (current_time>=10.0f && current_time<=13.0f){
            out_motors.m1_rads = 650;
            out_motors.m2_rads = 650;
            out_motors.m3_rads = 650;
            out_motors.m4_rads = 650;
        }

        if(current_time>=13.0f){
            out_motors.m1_rads = 633;
            out_motors.m2_rads = 633;
            out_motors.m3_rads = 633;
            out_motors.m4_rads = 633;
        }

        if (current_time>=25.0f){
            break;
        }

        simulation_link.send_motors(out_motors);

        // --- CLEAN TABLE LOGGING ---
        if (tick_counter % 100 == 0) {
            // Pre-calculate degrees to keep the stream injection clean
            float r_deg = current_state.current_angle.x * 180.0f / 3.14159f;
            float p_deg = current_state.current_angle.y * 180.0f / 3.14159f;
            float y_deg = current_state.current_angle.z * 180.0f / 3.14159f;

            // Force fixed-point notation for alignment
            std::cout << std::fixed << std::setprecision(2) << std::left 
                      << std::setw(8) << current_time << "|"
                      
                      << std::right
                      << std::setw(7) << r_deg << " " 
                      << std::setw(7) << p_deg << " " 
                      << std::setw(7) << y_deg << " |"
                      
                      << std::setw(7) << current_state.current_rate.x << " " 
                      << std::setw(7) << current_state.current_rate.y << " " 
                      << std::setw(7) << current_state.current_rate.z << " |"
                      
                      << std::setw(6) << torque_cmd.x << " " 
                      << std::setw(6) << torque_cmd.y << " " 
                      << std::setw(6) << torque_cmd.z << " |"
                      
                      << std::setw(8) << std::setprecision(3) << dynamic_thrust_N << " |"
                      
                      << std::setprecision(0) // Rads/s don't need decimals for visual monitoring
                      << std::setw(5) << out_motors.m1_rads << " "
                      << std::setw(5) << out_motors.m2_rads << " "
                      << std::setw(5) << out_motors.m3_rads << " "
                      << std::setw(5) << out_motors.m4_rads
                      << "\n";
        }
        
        tick_counter++;
        while (std::chrono::steady_clock::now() < next_tick) {
        }
    }
    return 0;
}