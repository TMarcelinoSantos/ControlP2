#include "control_p2/control_node.hpp"

using std::placeholders::_1;

ControlP2::ControlP2() : Node("control_node")
{
    /*------------------------------------------------------------------------------*/
    /*                                    TOPICS                                    */
    /*------------------------------------------------------------------------------*/

    this->declare_parameter(PARAMS_TOPIC_PATH, "/planned_path_topic");
	this->get_parameter(PARAMS_TOPIC_PATH, path_topic);

    this->declare_parameter(PARAMS_TOPIC_SPEED, "/acu_origin/dynamics");
	this->get_parameter(PARAMS_TOPIC_SPEED, rpm_topic);

    this->declare_parameter(PARAMS_TOPIC_DYNAMICS_CMD, "pc_origin/dynamics");
	this->get_parameter(PARAMS_TOPIC_DYNAMICS_CMD, dynamics_cmd_topic);

    this->declare_parameter(PARAMS_TOPIC_STATE, "/pc_origin/system_status/critical_as/state");
    this->get_parameter(PARAMS_TOPIC_STATE, state_topic);

    this->declare_parameter(PARAMS_TOPIC_MISSION, "pc_origin/system_status/critical_as/mission");
    this->get_parameter(PARAMS_TOPIC_MISSION, mission_topic);


    /*------------------------------------------------------------------------------*/
    /*                                   PUBLISHERS                                 */
    /*------------------------------------------------------------------------------*/
    
    dynamics_publisher = this->create_publisher<lart_msgs::msg::DynamicsCMD>(dynamics_cmd_topic, 10);

    /*------------------------------------------------------------------------------*/
    /*                                  SUBSCRIBERS                                 */
    /*------------------------------------------------------------------------------*/

    subscription_path = this->create_subscription<lart_msgs::msg::PathSpline>(
        path_topic, 10, std::bind(&SpacNode::path_callback, this, _1));

    subscription_speed = this->create_subscription<lart_msgs::msg::Dynamics>(
        current_speed_topic, 10, std::bind(&SpacNode::speed_callback, this, _1));

    state_subscriber = this->create_subscription<lart_msgs::msg::State>(
        state_topic, 10, std::bind(&SpacNode::state_callback, this, _1));

    mission_subscriber = this->create_subscription<lart_msgs::msg::Mission>(
        mission_topic, 10, std::bind(&SpacNode::mission_callback, this, _1));

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlP2>());
    rclcpp::shutdown();
    return 0;
}