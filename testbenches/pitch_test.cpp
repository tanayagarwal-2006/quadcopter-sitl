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

    vector2 angle_kp = {3.5, 3.5f}; 
    vector3 max_rates = {3.14f, 3.14f, 2.0f}; 
    angle_controller angle_ctrl(angle_kp, max_rates, 45.0f);

    PID_parameters roll_pid = {2.0f, 0.3f, 0.12f, 10.0f}; 
    PID_parameters pitch_pid = {2.0f, 0.3f, 0.12f, 10.0f};
    PID_parameters yaw_pid = {1.5f, 0.2f, 0.0f, 10.0f};

    rate_controller rate_ctrl(roll_pid, pitch_pid, yaw_pid, dt);

    motor_mixer mixer(0.0f, 1150); 

    state_payload current_state = {{0,0,0}, {0,0,0}};
    float MAX_RADS = 1150.0f; 

    StateEstimator estimator(dt, 0.999f);
    
    float base_hover_thrust = 1.2f * 9.81f;
    long long tick_counter = 0;

    // TABLE HEADER 
    std::cout << "\n===============================================================================================================================================================================\n";
    std::cout << std::left
            << std::setw(8)  << "TIME"
    
            << std::setw(22) << "DES ANG(R/P/Y)"
            << std::setw(22) << "EST ANG(R/P/Y)"
            << std::setw(16) << "ANG ERR"
    
            << std::setw(20) << "DES RATE(R/P/Y)"
            << std::setw(20) << "MEAS RATE(R/P/Y)"
            << std::setw(12) << "RATE ERR"
    
            << std::setw(10) << "TORQUE"
            << std::setw(26) << "MOTORS"
    
            << "\n";

    std::cout << "=========================================================================================================================\n";

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
        vector3 current_rate = {
            current_state.gyroscope.x,  
            current_state.gyroscope.y, 
            current_state.gyroscope.z 
        };

        if (current_time >= 10.0f) {
            float target_roll = 15.0f * (3.14159f / 180.0f);
            float slew_rate = 5.0f * (3.14159f / 180.0f) * dt;
            if (pilot_setpoint.y < target_roll) pilot_setpoint.y += slew_rate;
        }


        vector3 rate_cmd = angle_ctrl.update(pilot_setpoint, current_angle);

        vector3 angle_error = {
            pilot_setpoint.x - current_angle.x,
            pilot_setpoint.y - current_angle.y,
            pilot_setpoint.z - current_angle.z
        };

        vector3 torque_cmd = rate_ctrl.update(rate_cmd, current_rate);

        vector3 rate_error = {
            rate_cmd.x - current_rate.x,
            rate_cmd.y - current_rate.y,
            rate_cmd.z - current_rate.z
        };

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

            if (tick_counter % 100 == 0){
                auto deg = [](float rad)
                {
                    return rad * 180.0f / 3.14159265f;
                };

                std::cout
                    << std::fixed << std::setprecision(2)

                    << std::setw(7) << current_time << " | "

                    // Desired attitude
                    << std::setw(6) << deg(pilot_setpoint.x) << " "
                    << std::setw(6) << deg(pilot_setpoint.y) << " "
                    << std::setw(6) << deg(pilot_setpoint.z) << " | "

                    // Estimated attitude
                    << std::setw(6) << deg(current_angle.x) << " "
                    << std::setw(6) << deg(current_angle.y) << " "
                    << std::setw(6) << deg(current_angle.z) << " | "

                    // Angle error
                    << std::setw(6) << deg(angle_error.x) << " "
                    << std::setw(6) << deg(angle_error.y) << " | "

                    // Desired rates
                    << std::setw(6) << rate_cmd.x << " "
                    << std::setw(6) << rate_cmd.y << " "
                    << std::setw(6) << rate_cmd.z << " | "

                    // Measured rates
                    << std::setw(6) << current_rate.x << " "
                    << std::setw(6) << current_rate.y << " "
                    << std::setw(6) << current_rate.z << " | "

                    // Rate error
                    << std::setw(6) << rate_error.x << " "
                    << std::setw(6) << rate_error.y << " | "

                    // Torque
                    << std::setw(7) << torque_cmd.x << " | "

                    // Motors
                    << std::setw(4) << out_motors.m1_rads << " "
                    << std::setw(4) << out_motors.m2_rads << " "
                    << std::setw(4) << out_motors.m3_rads << " "
                    << std::setw(4) << out_motors.m4_rads

                    << "\n";
            }

        }

        if (current_time >= 45.0f) {
            std::cout << "\nPitch test complete.\n";
            break;
        }
        
        tick_counter++;
    }
    return 0;
}