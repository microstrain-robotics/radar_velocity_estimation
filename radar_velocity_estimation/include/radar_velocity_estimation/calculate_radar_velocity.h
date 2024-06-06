/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c)  2024, Microstrain by HBK
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CALCULATE_RADAR_VELOCITY_H
#define CALCULATE_RADAR_VELOCITY_H

#include <tuple>

#include "data.h"

namespace radar_velocity_estimation
{
    struct RadarVelocitySettings
    {
        double min_distance = 3.0;
        double inlier_range_from_median = 3.0;
        double min_point_cloud_size = 6.0;
    };

    std::tuple<bool, Eigen::Vector3d, Eigen::Matrix3d> caluclate_radar_velocity_with_covariance(const RadarPointCloud &radar_point_cloud, const RadarVelocitySettings &radar_velocity_settings);
}

#endif // CALCULATE_RADAR_VELOCITY_H
