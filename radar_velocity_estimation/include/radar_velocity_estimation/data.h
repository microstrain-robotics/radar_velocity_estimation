/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c)  2024, Microstrain by HBK
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RADAR_VELOCITY_ESTIMATION_DATA_H
#define RADAR_VELOCITY_ESTIMATION_DATA_H

#include <Eigen/Core>

namespace radar_velocity_estimation
{

    struct RadarPointCloud
    {
        double timestamp;
        std::vector<Eigen::Vector3d> points;
        std::vector<double> speed;
        std::vector<double> power;
    };

}

#endif // RADAR_VELOCITY_ESTIMATION_DATA_H
