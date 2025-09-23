#ifndef CONTROL_NODE_H_
#define CONTROL_NODE_H_

/*------------------------------------------------------------------------------*/
/*                                    INCULES                                   */
/*------------------------------------------------------------------------------*/

#include "rclcpp/rclcpp.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"

/*------------------------------------------------------------------------------*/
/*                                    DEFINES                                   */
/*------------------------------------------------------------------------------*/

#define PARAMS_TOPIC_PATH "path_topic"
#define PARAMS_TOPIC_SPEED "current_speed_topic"
#define PARAMS_TOPIC_DYNAMICS_CMD "dynamics_cmd_topic"
#define PARAMS_TOPIC_STATE "state_topic"
#define PARAMS_TOPIC_MISSION "mission_topic"

class ControlP2 : public rclcpp::Node
{
public:

    ControlP2();

private:


protected:
    // Subscribers
    std::string path_topic;
    std::string current_speed_topic;
    std::string state_topic;
    std::string mission_topic;

    // Publishers
    std::string dynamics_cmd_topic;

};

#endif