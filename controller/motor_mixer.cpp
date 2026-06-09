#include "motor_mixer.h"
#include "utils.h"
#include <algorithm>

motor_mixer::motor_mixer(float min_out, float max_out)
{
    min_output = min_out;   
    max_output = max_out;   
}

motor_mixer::motor_commands motor_mixer::mix(float thrust_N, const vector3& torque_Nm){
    motor_commands m;

    float kf = 7.34e-6f;
    float kt = 9.80e-8f;
    float L  = 0.225f * 0.7071f;  // effective arm

    // ---- convert to ω² space ----
    float base = thrust_N / (4.0f * kf);
    float roll  = torque_Nm.x / (4.0f * kf * L);
    float pitch = torque_Nm.y / (4.0f * kf * L);
    float yaw   = torque_Nm.z / (4.0f * kt);

    // ---- standard X configuration ----
    m.m1 = base - roll - pitch + yaw; // front right
    m.m2 = base + roll + pitch + yaw; // back left
    m.m3 = base + roll - pitch - yaw; // front left
    m.m4 = base - roll + pitch - yaw; // back right

    // ---- clamp ω² (physical limits) ----
    float max_sq = max_output * max_output;

    m.m1 = constrain_to_value(std::max(0.0f, m.m1), 0.0f, max_sq);
    m.m2 = constrain_to_value(std::max(0.0f, m.m2), 0.0f, max_sq);
    m.m3 = constrain_to_value(std::max(0.0f, m.m3), 0.0f, max_sq);
    m.m4 = constrain_to_value(std::max(0.0f, m.m4), 0.0f, max_sq);

    return m;
}