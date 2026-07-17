#include <cmath>
#include "state_estimator.h"
#include "vector_axes.h"
#include "utils.h"

StateEstimator::StateEstimator(float dt, float alpha):dt(dt),alpha(alpha){
    previous_attitude_measurement={0.0f, 0.0f, 0.0f};
    current_attitude_measurement={0.0f, 0.0f, 0.0f};
    accel_filtered={0.0f, 0.0f, 0.0f};
    previous_vel_world={0.0f, 0.0f, 0.0f};
}

vector3 StateEstimator::update(const vector3& gyro_data, const vector3& accel_data, const vector3& vel_world, float current_yaw){
    //Low pass filter for accelerometer
    float beta=0.95f;
    accel_filtered.x = beta * accel_filtered.x + (1.0f - beta) * accel_data.x;
    accel_filtered.y = beta * accel_filtered.y + (1.0f - beta) * accel_data.y;
    accel_filtered.z = beta * accel_filtered.z + (1.0f - beta) * accel_data.z;

    // Estimate translational acceleration from the simulator ground-truth velocity.
    vector3 acc_kin_world = {
        (vel_world.x - previous_vel_world.x) / dt,
        (vel_world.y - previous_vel_world.y) / dt,
        (vel_world.z - previous_vel_world.z) / dt
    };
    previous_vel_world = vel_world;

    float roll_prev=previous_attitude_measurement.x;
    float pitch_prev=previous_attitude_measurement.y;

    // Convert world-frame translational acceleration into the body frame
    // using the current attitude estimate. This acceleration is removed
    // from the accelerometer measurement to isolate gravity.
    float sin_roll=std::sin(roll_prev);
    float cos_roll=std::cos(roll_prev);
    float sin_pitch=std::sin(pitch_prev);
    float cos_pitch=std::cos(pitch_prev);
    float sin_yaw=std::sin(current_yaw);
    float cos_yaw=std::cos(current_yaw);
    float tan_pitch=std::tan(pitch_prev);

    float a_body_x = acc_kin_world.x*(cos_pitch*cos_yaw) + acc_kin_world.y*(cos_pitch*sin_yaw) - acc_kin_world.z*(sin_pitch);
    float a_body_y = acc_kin_world.x*(sin_roll*sin_pitch*cos_yaw - cos_roll*sin_yaw) + 
                     acc_kin_world.y*(sin_roll*sin_pitch*sin_yaw + cos_roll*cos_yaw) + 
                     acc_kin_world.z*(sin_roll*cos_pitch);
    float a_body_z = acc_kin_world.x*(cos_roll*sin_pitch*cos_yaw + sin_roll*sin_yaw) + 
                     acc_kin_world.y*(cos_roll*sin_pitch*sin_yaw - sin_roll*cos_yaw) + 
                     acc_kin_world.z*(cos_roll*cos_pitch);


    // Remove linear acceleration from the accelerometer.
    // After compensation, the remaining measurement approximates gravity.
    float ax_corr = accel_filtered.x - a_body_x;
    float ay_corr = accel_filtered.y - a_body_y;
    float az_corr = accel_filtered.z - a_body_z;

    //Need to add magnetometer model for proper yaw measurement

    //Kinematic transformation
    float roll_rate_euler=gyro_data.x+(sin_roll*tan_pitch*gyro_data.y)+(cos_roll*tan_pitch*gyro_data.z);
    float pitch_rate_euler=(cos_roll*gyro_data.y)-(sin_roll*gyro_data.z);

    float roll_angle_gyro = roll_prev + roll_rate_euler * dt;
    float pitch_angle_gyro = pitch_prev + pitch_rate_euler * dt;

    float roll_acc = std::atan2(-ay_corr, -az_corr);
    float pitch_acc = std::atan2(ax_corr, std::sqrt(ay_corr*ay_corr + az_corr*az_corr));

    current_attitude_measurement.x=alpha*roll_angle_gyro+(1.0f-alpha)*roll_acc;
    current_attitude_measurement.y=alpha*pitch_angle_gyro+(1.0f-alpha)*pitch_acc;

    current_attitude_measurement.z=0.0f;

    previous_attitude_measurement=current_attitude_measurement;

    return current_attitude_measurement;
}