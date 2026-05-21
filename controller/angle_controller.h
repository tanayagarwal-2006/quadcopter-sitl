#ifndef ANGLE_CONTROLLER_H
#define ANGLE_CONTROLLER_H

#include <cmath>
#include "vector_axes.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

class angle_controller{
private:
    vector2 K;
    vector3 max_rate;
    float max_angle;
    float angle_correction(float angle);

public:
    angle_controller(vector2 kp, vector3 maximum_rate, float max_angle_degrees);
    vector3 update(const vector3& desired_input, const vector3& current_angle);
};
#endif