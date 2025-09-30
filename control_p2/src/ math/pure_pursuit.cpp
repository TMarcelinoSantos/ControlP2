#include "control_p2/math/pure_pursuit.hpp"

Pursuit_Algorithm::Pursuit_Algorithm(){
    // Constructor
}

Pursuit_Algorithm::calculate_control(lart_msgs::msg::PathSpline path, geometry_msgs::msg::PoseStamped pose,
             float current_speed, float current_steering, float mission_speed){

    // Calculate control commands using pure pursuit algorithm
}
