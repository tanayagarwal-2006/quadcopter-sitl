#ifndef MOTOR_MIXER_H
#define MOTOR_MIXER_H

#include "vector_axes.h"

class motor_mixer{
    private:
        float min_output;
        float max_output;

    public:
        struct motor_commands{
            float m1;
            float m2;
            float m3;
            float m4;
        };
        motor_mixer(float min_out=0.0f, float max_out=1.0f);
        motor_commands mix(float throttle, const vector3& torque);
};

#endif