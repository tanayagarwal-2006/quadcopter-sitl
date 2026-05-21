#include "motor_mixer.h"
#include "utils.h"
#include <algorithm>

motor_mixer::motor_mixer(float min_out, float max_out){
    min_output=min_out;
    max_output=max_out;
}

motor_mixer::motor_commands motor_mixer::mix(float throttle, const vector3& torque){
    motor_commands motor;

    float roll=torque.x;
    float pitch=torque.y;
    float yaw=torque.z;

    // Motor-1: Front right, Motor-2: Back left, Motor-3: Front left, Motor-4: Back right
    motor.m1 = throttle - roll - pitch + yaw;
    motor.m2 = throttle + roll + pitch + yaw;
    motor.m3 = throttle + roll - pitch - yaw;
    motor.m4 = throttle - roll + pitch - yaw;
    
    float min_motor = std::min({motor.m1, motor.m2, motor.m3, motor.m4});
    float max_motor = std::max({motor.m1, motor.m2, motor.m3, motor.m4});

    if (min_motor<min_output){
        float diff=min_output-min_motor;
        motor.m1+=diff;
        motor.m2+=diff;
        motor.m3+=diff;
        motor.m4+=diff;
    }

    max_motor = std::max({motor.m1, motor.m2, motor.m3, motor.m4});

    if (max_motor > max_output) {
        float shift = max_motor - max_output;
        motor.m1 -= shift; 
        motor.m2 -= shift; 
        motor.m3 -= shift; 
        motor.m4 -= shift;
    }

    motor.m1 = constrain_to_value(motor.m1, min_output, max_output);
    motor.m2 = constrain_to_value(motor.m2, min_output, max_output);
    motor.m3 = constrain_to_value(motor.m3, min_output, max_output);
    motor.m4 = constrain_to_value(motor.m4, min_output, max_output);

    return motor;
}