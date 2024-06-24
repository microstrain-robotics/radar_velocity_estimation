/////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c)  2024, Microstrain by HBK
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"

#include "radar_velocity_estimation/calculate_radar_velocity.h"

#include "RadialVelocityFactor.h"

namespace radar_velocity_estimation
{

    double find_median(std::vector<double> speeds)
    {
        std::sort(speeds.begin(), speeds.end());
        int length = speeds.size();
        double median = 0.0;

        median = (speeds[(length - 1) / 2] + speeds[length / 2]) / 2;

        if (length % 2 != 0)
            median = speeds[length / 2];

        return median;
    }

    RadarPointCloud reject_outliers(const RadarPointCloud &radar_point_cloud, double range)
    {
        std::vector<double> speeds;
        for (size_t i = 0; i < radar_point_cloud.speed.size(); i++)
        {
            speeds.push_back(-1 * radar_point_cloud.speed[i]);
        }

        double median = find_median(speeds);

        RadarPointCloud radar_point_cloud_rej;

        for (size_t i = 0; i < speeds.size(); i++)
        {
            if ((median - range) < speeds[i] && speeds[i] < (median + range))
            {
                radar_point_cloud_rej.speed.push_back(speeds[i]);
                radar_point_cloud_rej.points.push_back(radar_point_cloud.points[i]);
            }
        }

        return radar_point_cloud_rej;
    }

    std::tuple<bool, Eigen::Vector3d, Eigen::Matrix3d> caluclate_radar_velocity_with_covariance(const RadarPointCloud &radar_point_cloud, const RadarVelocitySettings &radar_velocity_settings)
    {

        Eigen::Vector3d velocity_estimate{0, 0, 0};
        Eigen::Matrix3d velocity_covariance = Eigen::Matrix3d::Zero();
        bool solution_valid = false;

        //  Remove explicit outlier from point cloud
        RadarPointCloud radar_point_cloud_rej = reject_outliers(radar_point_cloud, radar_velocity_settings.inlier_range_from_median);

        // Minimum number of points required for velocity vector to be observable
        // Just return if minimum threshold is not met
        if (radar_point_cloud.points.size() < radar_velocity_settings.min_point_cloud_size)
        {
            return {solution_valid, velocity_estimate, velocity_covariance};
        }

        auto radial_velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(1, .25);

        // Robust models are applied "sequentially", each step with more agressive outlier rejection model
        // This allows for good initial convergence while still virtually eliminating large outliers
        std::vector<gtsam::noiseModel::Robust::shared_ptr> robust_radial_velocity_noise_models;
        robust_radial_velocity_noise_models.push_back(gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(3), radial_velocity_noise_model));
        robust_radial_velocity_noise_models.push_back(gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Welsch::Create(3), radial_velocity_noise_model));

        gtsam::Key velocity_key = 0;
        double median = find_median(radar_point_cloud_rej.speed);
        velocity_estimate = {median, 0, 0};

        for (size_t model = 0; model < robust_radial_velocity_noise_models.size(); model++)
        {
            gtsam::NonlinearFactorGraph factors;
            gtsam::Values states;
            states.insert(velocity_key, velocity_estimate);

            // Build factor graph
            for (size_t i = 0; i < radar_point_cloud_rej.points.size(); i++)
            {
                // Check minimum distance threshold
                if (radar_point_cloud_rej.points[i].norm() < radar_velocity_settings.min_distance)
                {
                    continue;
                }

                auto radar_factor = boost::make_shared<BodyframeVelocityFactor>(velocity_key, radar_point_cloud_rej.points[i], radar_point_cloud_rej.speed[i], robust_radial_velocity_noise_models[model]);
                factors.add(radar_factor);
            }

            // Don't attempt to solve if no factors are created
            if (factors.size() == 0)
            {
                return {solution_valid, velocity_estimate, velocity_covariance};
            }

            // Solve
            try
            {
                gtsam::LevenbergMarquardtOptimizer optimizer(factors, states);
                gtsam::Values solution = optimizer.optimize();

                velocity_estimate = solution.at<gtsam::Vector3>(velocity_key);

                // Only calculate covariance on the final step
                if (model == (robust_radial_velocity_noise_models.size() - 1))
                {
                    gtsam::Marginals marginals(factors, solution);
                    velocity_covariance = marginals.marginalCovariance(velocity_key);
                    solution_valid = true;
                }
            }
            catch (const gtsam::IndeterminantLinearSystemException &e)
            {
            }
        }

        // Scale covariance to appropriate value (determined experimentally)
        double target_x_covariance = pow(.15, 2);
        double covariance_scaling = target_x_covariance / velocity_covariance(0, 0);
        velocity_covariance *= covariance_scaling;

        return {solution_valid, velocity_estimate, velocity_covariance};
    }
}
