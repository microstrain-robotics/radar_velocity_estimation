/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c)  2024, Microstrain by HBK
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RADARVELOCITYNODE_H
#define RADARVELOCITYNODE_H

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "std_srvs/srv/trigger.hpp"

namespace radar_velocity_estimation_node
{

    class RadarVelocityNode : public rclcpp::Node
    {
    public:

        RadarVelocityNode(const std::string &node_name = "radar_velocity_node");

    protected:
        void handle_input_point_cloud(sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg);

        double get_min_point_distance() { return this->get_parameter("min_point_distance").as_double(); };

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _point_cloud_subscriber;

        // Publishers
        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr _radar_velocity_publisher;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _radar_velocity_viz_publisher;
    };
} // radar_velocity_estimation_node

#endif // RADARVELOCITYNODE_H
