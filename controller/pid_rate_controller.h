#ifndef PID_RATE_CONTROLLER_H
#define PID_RATE_CONTROLLER_H

#include "vector_axes.h"

struct PID_parameters{
    float kp;
    float ki;
    float kd;
    float i_max;
};

class rate_controller{
    private:
        PID_parameters roll_parameters;
        PID_parameters pitch_parameters;
        PID_parameters yaw_parameters;

        vector3 integral;
        vector3 prev_rate;

        float dt;
        
        bool integral_windup_prevent(float error, float theoretical_torque, float max_torque);

    public:
        rate_controller(PID_parameters roll, PID_parameters pitch, PID_parameters yaw, float timestep);
        vector3 update(const vector3& desired_rates, const vector3& measured_rates);
};

#endif