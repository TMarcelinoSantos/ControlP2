#ifndef TARGET_H_
#define TARGET_H_

#include "math/pure_pursuit.hpp"
#include "utils.hpp"


class Target{
    public:
        Target(float mission_speed);
    private:
        Pursuit_Algorithm algorithm;
    protected:
        // Parameters
        float mission_speed;
        bool ready = false;
        bool mission_set = false;

    
};

#endif