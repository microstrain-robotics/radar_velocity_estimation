/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c)  2024, Microstrain by HBK
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROS_RADAR_UTILS_H
#define ROS_RADAR_UTILS_H

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "radar_velocity_estimation/data.h"

namespace radar_velocity_estimation_node
{
    radar_velocity_estimation::RadarPointCloud convert_from_ros_message(const sensor_msgs::msg::PointCloud2 &point_cloud2_message);

    template <int rows, int cols>
    std::array<double, rows * cols> from_eigen_covariance(const Eigen::Matrix<double, rows, cols> covariance)
    {
        std::array<double, rows * cols> covariance_vector;
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                covariance_vector[cols * i + j] = covariance(i, j);
        return covariance_vector;
    }
}

#endif // ROS_RADAR_UTILS_H
