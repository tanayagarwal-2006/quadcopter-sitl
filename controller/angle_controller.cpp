#include "angle_controller.h"
#include "utils.h"

angle_controller::angle_controller(vector2 kp, vector3 maximum_rate, float max_angle_degrees)
: K(kp), max_rate(maximum_rate)
{
    max_angle = max_angle_degrees * M_PI / 180.0f;
}

float angle_controller::angle_correction(float angle) {
    angle = fmod(angle + M_PI, 2.0f * M_PI);
    if (angle < 0.0f) angle += 2.0f * M_PI;
    return angle - M_PI;
}

vector3 angle_controller::update(const vector3& desired_input, const vector3& current_angle){
    vector3 angle_error;
    vector3 desired_angular_rate;
    vector3 desired_angle;
    float yaw_rate_command;

    yaw_rate_command=desired_input.z;

    desired_angle.x = constrain_to_value(desired_input.x, -max_angle, max_angle);
    angle_error.x = angle_correction(desired_angle.x - current_angle.x);
    desired_angular_rate.x = constrain_to_value(K.x * angle_error.x, -max_rate.x, max_rate.x);

    desired_angle.y = constrain_to_value(desired_input.y, -max_angle, max_angle);
    angle_error.y = angle_correction(desired_angle.y - current_angle.y);
    desired_angular_rate.y = constrain_to_value(K.y * angle_error.y, -max_rate.y, max_rate.y);

    desired_angular_rate.z = constrain_to_value(yaw_rate_command, -max_rate.z, max_rate.z);

    return desired_angular_rate;
}