#ifndef UTILS_H_
#define UTILS_H_

/*------------------------------------------------------------------------------*/
/*                                   INCLUDES                                   */
/*------------------------------------------------------------------------------*/

#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/path_spline.hpp"
#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"

/*------------------------------------------------------------------------------*/
/*                                   TOPICS                                     */
/*------------------------------------------------------------------------------*/

#define TOPIC_PATH "/planned_path_topic"
#define TOPIC_SPEED "/acu_origin/dynamics"
#define TOPIC_DYNAMICS_CMD "pc_origin/dynamics"
#define TOPIC_STATE "/pc_origin/system_status/critical_as/state"
#define TOPIC_MISSION "pc_origin/system_status/critical_as/mission"
#define TOPIC_SLAM "/ekf/state"

#endif