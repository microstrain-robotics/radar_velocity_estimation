/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c)  2024, Microstrain by HBK
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef TOPICS_H
#define TOPICS_H

#include <string>

namespace radar_velocity_estimation_node::topics
{
    const std::string ROS_INPUT_RADAR_TOPIC_NAME = "smart_radar/can_targets_0";

    const std::string ROS_OUTPUT_RADAR_VELOCITY_TOPIC_NAME = "cv7_ins/ext/velocity_body";
    const std::string ROS_OUTPUT_RADAR_VELOCITY_VIZ_TOPIC_NAME = "radar_velocity_viz";

    const std::vector TOPICS_TO_RECORD = {ROS_OUTPUT_RADAR_VELOCITY_TOPIC_NAME};
}

#endif // TOPICS_H
