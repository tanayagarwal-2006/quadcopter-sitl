#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include "vector_axes.h"
#include "utils.h"

class StateEstimator{
    private:
        vector3 previous_attitude_measurement;
        vector3 current_attitude_measurement;
        vector3 imu_data;

        vector3 accel_filtered;

        vector3 previous_vel_world;

        float dt;
        float alpha;

     public:
        StateEstimator(float dt, float alpha = 0.98f);
        vector3 update(const vector3& gyro_data, const vector3& accel_data, const vector3& vel_world, float current_yaw);
};

#endif