#include "udp_bridge.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <windows.h>
#include "vector_axes.h"
#include "utils.h"
#include "angle_controller.h"
#include "pid_rate_controller.h"
#include "motor_mixer.h"
#include "state_estimator.h"


int main() {
    float dt = 0.001f; 

    udp_bridge simulation_link("127.0.0.1", 5000, 5001);
    std::cout << "C++ Brain Online (1000Hz). Waiting for SITL...\n";

    vector2 angle_kp = {4.5f, 4.5f}; 
    vector3 max_rates = {3.14f, 3.14f, 2.0f}; 
    angle_controller angle_ctrl(angle_kp, max_rates, 45.0f);

    PID_parameters roll_pid = {1.4f, 0.2f, 0.04f, 10.0f}; 
    PID_parameters pitch_pid = {1.4f, 0.2f, 0.04f, 10.0f};
    PID_parameters yaw_pid = {1.5f, 0.2f, 0.0f, 10.0f};

    rate_controller rate_ctrl(roll_pid, pitch_pid, yaw_pid, dt);

    motor_mixer mixer(0.0f, 1150); 

    state_payload current_state = {{0,0,0}, {0,0,0}};
    float MAX_RADS = 1150.0f; 

    StateEstimator estimator(dt, 0.99f);
    
    float base_hover_thrust = 1.2f * 9.81f;
    long long tick_counter = 0;

    // TABLE HEADER 
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

    vector3 pilot_setpoint = {0.0f, 0.0f, 0.0f};

    while (true) {
        bool received = simulation_link.receive_state(current_state);
        if (!received) {
            continue;
        }

        float current_time = tick_counter * dt;

        motor_payload out_motors;

        vector3 estimated_angles=estimator.update(current_state.gyroscope,current_state.accelerometer, 
            current_state.velocity, current_state.yaw_angle);
        vector3 current_angle={estimated_angles.x, estimated_angles.y, current_state.yaw_angle};
        vector3 current_rate={current_state.gyroscope.x, current_state.gyroscope.y, current_state.gyroscope.z};

        if (current_time >= 5.0f) { 
            // 1. Pitch forward to generate forward flight speed (8 degrees)
            pilot_setpoint.y = 8.0f * (3.14159f / 180.0f); 
            
            // 2. Roll to bank into the circular turn (15 degrees)
            pilot_setpoint.x = 15.0f * (3.14159f / 180.0f); 
            
            pilot_setpoint.z=0.5f;
        }

        vector3 rate_cmd = angle_ctrl.update(pilot_setpoint, current_angle);
        vector3 torque_cmd = rate_ctrl.update(rate_cmd, current_rate);

        // FEED-FORWARD THROTTLE COMPENSATION
        float current_roll = current_angle.x;
        float current_pitch = current_angle.y;

        float max_ff_angle = 0.698f;
        float eff_roll = constrain_to_value(std::abs(current_roll), 0.0f, max_ff_angle);
        float eff_pitch = constrain_to_value(std::abs(current_pitch), 0.0f, max_ff_angle);

        float dynamic_thrust_N = (base_hover_thrust / (std::cos(eff_roll) * std::cos(eff_pitch)));

        motor_mixer::motor_commands motors = mixer.mix(dynamic_thrust_N, torque_cmd);

        out_motors.m1_rads = std::sqrt(constrain_to_value(motors.m1, 0.0f, MAX_RADS * MAX_RADS));
        out_motors.m2_rads = std::sqrt(constrain_to_value(motors.m2, 0.0f, MAX_RADS * MAX_RADS));
        out_motors.m3_rads = std::sqrt(constrain_to_value(motors.m3, 0.0f, MAX_RADS * MAX_RADS));
        out_motors.m4_rads = std::sqrt(constrain_to_value(motors.m4, 0.0f, MAX_RADS * MAX_RADS));

        if (current_time < 2.0f) {
            // Sit on the pad / spool up
            out_motors.m1_rads = 633; out_motors.m2_rads = 633;
            out_motors.m3_rads = 633; out_motors.m4_rads = 633;
        } 
        else if (current_time >= 2.0f && current_time < 5.0f) {
            // Punch it to gain altitude
            out_motors.m1_rads = 650; out_motors.m2_rads = 650;
            out_motors.m3_rads = 650; out_motors.m4_rads = 650;
        }
 
        simulation_link.send_motors(out_motors);

        if (tick_counter % 100 == 0) {
            float r_deg = current_angle.x * 180.0f / 3.14159f;
            float p_deg = current_angle.y * 180.0f / 3.14159f;
            float y_deg = current_angle.z * 180.0f / 3.14159f;

            std::cout << std::fixed << std::setprecision(2) << std::left 
                      << std::setw(8) << current_time << "|"
                      
                      << std::right
                      << std::setw(7) << r_deg << " " 
                      << std::setw(7) << p_deg << " " 
                      << std::setw(7) << y_deg << " |"
                                    
                      << std::setw(7) << current_rate.x << " " 
                      << std::setw(7) << current_rate.y << " " 
                      << std::setw(7) << current_rate.z << " |"
                      
                      << std::setw(6) << torque_cmd.x << " " 
                      << std::setw(6) << torque_cmd.y << " " 
                      << std::setw(6) << torque_cmd.z << " |"
                      
                      << std::setw(8) << std::setprecision(3) << dynamic_thrust_N << " |"
                      
                      << std::setprecision(0) 
                      << std::setw(5) << out_motors.m1_rads << " "
                      << std::setw(5) << out_motors.m2_rads << " "
                      << std::setw(5) << out_motors.m3_rads << " "
                      << std::setw(5) << out_motors.m4_rads
                      << "\n";
        }
        
        tick_counter++;
    }
    return 0;
}