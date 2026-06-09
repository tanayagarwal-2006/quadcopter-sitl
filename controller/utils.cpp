#include "utils.h"

float constrain_to_value(float val, float min_val, float max_val){
    if(val > max_val){
        return max_val;
    }
    if(val < min_val){
        return min_val;
    }
    return val;
}