#include <cmath>
#include "state_estimator.h"
#include "vector_axes.h"
#include "utils.h"

StateEstimator::StateEstimator(float dt, float alpha):dt(dt),alpha(alpha){
    previous_attitude_measurement={0.0f, 0.0f, 0.0f};
    current_attitude_measurement={0.0f, 0.0f, 0.0f};
}

vector3 StateEstimator::update(const vector3& gyro_data, const vector3& accel_data){
    float roll_prev=previous_attitude_measurement.x;
    float pitch_prev=previous_attitude_measurement.y;
    //float yaw_prev=previous_attitude_measurement.z;

    //Need to add magnetometer model for proper yaw measurement

    float sin_roll=std::sin(roll_prev);
    float cos_roll=std::cos(roll_prev);
    float tan_pitch=std::tan(pitch_prev);


    //Kinematic transformation
    float roll_rate_euler=gyro_data.x+(sin_roll*tan_pitch*gyro_data.y)+(cos_roll*tan_pitch*gyro_data.z);
    float pitch_rate_euler=(cos_roll*gyro_data.y)-(sin_roll*gyro_data.z);

    //float roll_angle_gyro=roll_prev+gyro_data.x * dt;
    //float pitch_angle_gyro=pitch_prev+gyro_data.y * dt;
    //float yaw_angle_gyro=yaw_prev+gyro_data.z * dt;

    float roll_angle_gyro = roll_prev + roll_rate_euler * dt;
    float pitch_angle_gyro = pitch_prev + pitch_rate_euler * dt;

    float roll_acc=std::atan2(-accel_data.y, -accel_data.z);
    float pitch_acc=std::atan2(accel_data.x,std::sqrt(accel_data.y*accel_data.y+accel_data.z*accel_data.z));

    current_attitude_measurement.x=alpha*roll_angle_gyro+(1.0f-alpha)*roll_acc;
    current_attitude_measurement.y=alpha*pitch_angle_gyro+(1.0f-alpha)*pitch_acc;
    //current_attitude_measurement.z=yaw_angle_gyro;
    current_attitude_measurement.z=0.0f;

    previous_attitude_measurement=current_attitude_measurement;

    return current_attitude_measurement;
}