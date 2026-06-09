#include "pid_rate_controller.h"
#include "utils.h"
#include <cmath>

rate_controller::rate_controller(PID_parameters roll, PID_parameters pitch, PID_parameters yaw, float timestep){
    roll_parameters=roll;
    pitch_parameters=pitch;
    yaw_parameters=yaw;

    dt=timestep;

    integral={0,0,0};
    prev_rate={0,0,0};
}

bool rate_controller::integral_windup_prevent(float error, float theoretical_torque, float max_torque){
    bool saturation_reached = (std::abs(theoretical_torque)>=max_torque);
    bool pushing_wrong_way = (theoretical_torque>0.0f && error>0.0f) || (theoretical_torque<0.0f && error<0.0f);

    if (saturation_reached && pushing_wrong_way){
        return false;
    }
    return true;
}

vector3 rate_controller::update(const vector3& desired_rates, const vector3& measured_rates){
    vector3 torque_command;
    //float temp_max_axis_torque=5.0f; //Temporary hardcoded fix

    //Hardcoded for now
    float max_x_axis_torque=3.0f;
    float max_y_axis_torque=3.0f;
    float max_z_axis_torque=0.25f;

    //Roll
    float roll_error = desired_rates.x-measured_rates.x;
    float roll_p = roll_parameters.kp*roll_error;
    float roll_derivative = -(measured_rates.x - prev_rate.x)/dt;
    float roll_d = roll_parameters.kd*roll_derivative;

    float theoretical_roll_torque = roll_p+(roll_parameters.ki*integral.x)+roll_d;

    if(integral_windup_prevent(roll_error,theoretical_roll_torque,max_x_axis_torque)){
        integral.x += roll_error*dt;
    }

    integral.x = constrain_to_value(integral.x,-roll_parameters.i_max,roll_parameters.i_max);

    torque_command.x = roll_p + (roll_parameters.ki * integral.x) + roll_d;

    prev_rate.x = measured_rates.x;

    //Pitch
    float pitch_error = desired_rates.y-measured_rates.y;
    float pitch_p = pitch_parameters.kp*pitch_error;
    float pitch_derivative = -(measured_rates.y - prev_rate.y)/dt;
    float pitch_d = pitch_parameters.kd*pitch_derivative;

    float theoretical_pitch_torque = pitch_p+(pitch_parameters.ki*integral.y)+pitch_d;

    if(integral_windup_prevent(pitch_error, theoretical_pitch_torque, max_y_axis_torque)){
        integral.y += pitch_error*dt;
    }

    integral.y = constrain_to_value(integral.y,-pitch_parameters.i_max,pitch_parameters.i_max);

    torque_command.y = pitch_p + (pitch_parameters.ki * integral.y) + pitch_d;

    prev_rate.y = measured_rates.y;

    //Yaw
    float yaw_error = desired_rates.z - measured_rates.z;
    float yaw_p = yaw_parameters.kp * yaw_error;
    float yaw_derivative = -(measured_rates.z - prev_rate.z) / dt;
    float yaw_d = yaw_parameters.kd * yaw_derivative;

    float theoretical_yaw_torque = yaw_p + (yaw_parameters.ki * integral.z) + yaw_d;

    if (integral_windup_prevent(yaw_error, theoretical_yaw_torque, max_z_axis_torque)) {
        integral.z += yaw_error * dt;
    }
    
    integral.z = constrain_to_value(integral.z, -yaw_parameters.i_max, yaw_parameters.i_max);
    torque_command.z = yaw_p + (yaw_parameters.ki * integral.z) + yaw_d;
    prev_rate.z = measured_rates.z;

    torque_command.x = constrain_to_value(torque_command.x, -max_x_axis_torque, max_x_axis_torque);
    torque_command.y = constrain_to_value(torque_command.y, -max_y_axis_torque, max_y_axis_torque);
    torque_command.z = constrain_to_value(torque_command.z, -max_z_axis_torque, max_z_axis_torque);
         
    /*torque_command.x /= max_x_axis_torque;
    torque_command.y /= max_y_axis_torque;
    torque_command.z /= max_z_axis_torque;*/    
    return torque_command;
}