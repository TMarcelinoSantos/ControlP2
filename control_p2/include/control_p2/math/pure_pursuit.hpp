#ifndef PURE_PURSUIT_H_
#define PURE_PURSUIT_H_
#include "../utils.hpp"

class Pursuit_Algorithm {
    public:
        Pursuit_Algorithm();
        calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped pose,
             float current_speed, float current_steering, float mission_speed);
    private:

        
};

#endif