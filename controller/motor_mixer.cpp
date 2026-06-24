#include "motor_mixer.h"
#include "utils.h"
#include <algorithm>

motor_mixer::motor_mixer(float min_out, float max_out){
    min_output = min_out;   
    max_output = max_out;   
}

motor_mixer::motor_commands motor_mixer::mix(float thrust_N, const vector3& torque_Nm){
    motor_commands motor;

    float k_thrust = 7.34e-6f;
    float k_torque = 9.80e-8f;
    float L  = 0.225f * 0.7071f;

    // convert to ω² space 
    float base = thrust_N / (4.0f * k_thrust);
    float roll  = torque_Nm.x / (4.0f * k_thrust * L);
    float pitch = torque_Nm.y / (4.0f * k_thrust * L);
    float yaw   = torque_Nm.z / (4.0f * k_torque);

    // standard X configuration 
    motor.m1 = base - roll - pitch + yaw; // front right
    motor.m2 = base + roll + pitch + yaw; // back left
    motor.m3 = base + roll - pitch - yaw; // front left
    motor.m4 = base - roll + pitch - yaw; // back right

    // clamp ω² (physical limits) 
    float max_sq = max_output * max_output;

    motor.m1 = constrain_to_value(std::max(0.0f, motor.m1), 0.0f, max_sq);
    motor.m2 = constrain_to_value(std::max(0.0f, motor.m2), 0.0f, max_sq);
    motor.m3 = constrain_to_value(std::max(0.0f, motor.m3), 0.0f, max_sq);
    motor.m4 = constrain_to_value(std::max(0.0f, motor.m4), 0.0f, max_sq);

    return motor;
}