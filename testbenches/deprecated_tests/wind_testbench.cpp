#include <iostream>
#include <iomanip>
#include <cmath>
#include "vector_axes.h"
#include "angle_controller.h"
#include "pid_rate_controller.h"
#include "motor_mixer.h"

int main() {
    float dt = 0.01f; 
    float inertia_xy = 0.05f; 
    float inertia_z = 0.10f;  

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
    
    // --- NEW: Translational Kinematics Trackers ---
    vector2 position = {0.0f, 0.0f}; // X (Forward), Y (Right) in meters
    vector2 velocity = {0.0f, 0.0f}; // m/s
    float gravity = 9.81f;           // m/s^2

    vector3 pilot_setpoint = {0.0f, 0.0f, 0.0f};
    float base_hover_throttle = 0.5f; 

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time\t| Roll Ang\t| Pitch Ang\t| Pos Y (Left/Right)\t| Pos X (Fwd/Back)\n";
    std::cout << "------------------------------------------------------------------------------------------\n";

    float MAX_RADS = 1000.0f; 
    float K_torque = 1.0f / (4.0f * MAX_RADS * MAX_RADS);
    float K_yaw_torque = K_torque * 0.1f; 

    for (int i = 0; i <= 3000; ++i) { // 30 second simulation
        float time = i * dt;

        // --- INJECT 3D WEATHER ---
        vector3 wind_torque = {0.0f, 0.0f, 0.0f};

        // Event 1: Chaotic Microburst
        if (time >= 1.0f && time < 2.0f) {
            wind_torque.x = -0.15f * std::sin(time * 15.0f); 
            wind_torque.y = 0.10f * std::cos(time * 10.0f);  
            wind_torque.z = 0.02f * std::sin(time * 5.0f);   
        }
        // Event 2: Constant Steady Crosswind
        else if (time >= 2.0f) {
            wind_torque.x = -0.08f; 
            wind_torque.y = 0.05f;  
            wind_torque.z = 0.00f;
        }

        // --- CASCADED FLIGHT CONTROLLER PIPELINE ---
        vector3 rate_cmd = angle_ctrl.update(pilot_setpoint, current_angle);
        vector3 torque_cmd = rate_ctrl.update(rate_cmd, current_rate);
        motor_mixer::motor_commands motors = mixer.mix(base_hover_throttle, torque_cmd);

        float w1 = MAX_RADS * std::sqrt(motors.m1); 
        float w2 = MAX_RADS * std::sqrt(motors.m2); 
        float w3 = MAX_RADS * std::sqrt(motors.m3); 
        float w4 = MAX_RADS * std::sqrt(motors.m4); 

        // --- 3D ROTATIONAL PHYSICS ---
        float generated_roll  = K_torque * ((w2*w2 + w3*w3) - (w1*w1 + w4*w4));
        float generated_pitch = K_torque * ((w2*w2 + w4*w4) - (w1*w1 + w3*w3));
        float generated_yaw   = K_yaw_torque * ((w1*w1 + w2*w2) - (w3*w3 + w4*w4));

        current_rate.x += ((generated_roll + wind_torque.x) / inertia_xy) * dt;
        current_rate.y += ((generated_pitch + wind_torque.y) / inertia_xy) * dt;
        current_rate.z += ((generated_yaw + wind_torque.z) / inertia_z) * dt;

        current_angle.x += current_rate.x * dt;
        current_angle.y += current_rate.y * dt;
        current_angle.z += current_rate.z * dt; 

        // --- NEW: TRANSLATIONAL PHYSICS (Drift) ---
        // Calculate lateral acceleration based on the drone's tilt angle
        float accel_x = gravity * std::sin(current_angle.y); // Pitch (Nose down = Forward)
        float accel_y = gravity * std::sin(current_angle.x); // Roll (Tilt Right = Right)

        velocity.x += accel_x * dt;
        velocity.y += accel_y * dt;
        
        position.x += velocity.x * dt;
        position.y += velocity.y * dt;

        // Print outputs every 0.1s
        if (i % 10 == 0) {
            std::cout << time << "\t| " 
                      << current_angle.x * 180.0f / 3.14159f << "\t\t| " 
                      << current_angle.y * 180.0f / 3.14159f << "\t\t| " 
                      << position.y << " m\t\t| "
                      << position.x << " m\n";
        }
    }
    return 0;
}