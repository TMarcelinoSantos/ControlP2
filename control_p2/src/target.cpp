#include "control_p2/target.hpp"

Target::Target(float mission_speed){
    this->algorithm = Pursuit_Algorithm();
    this->mission_speed = mission_speed;
}

lart_msgs::msg::DynamicsCMD Target::getDynamicsCMD(){
    
    // CODE FOR CALCULATING THE DYNAMICSCMD

    // CALLS MULTIPLE FUNCTIONS ACORDING TO THE NECESSITIES OF THE FLAGS
    
}

// Add a lot of diferent options that can be activated with flags
// Mostly an option to have a difrent aproach to the speed control
// One in wich the speed control is expected from the path planner
// And another where the speed control is done by the control node