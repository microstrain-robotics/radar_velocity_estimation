//
// Created by davidrobbins on 11/28/23.
//

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/Marginals.h"

#include "radar_velocity_estimation/calculate_radar_velocity.h"

#include "RadialVelocityFactor.h"

namespace radar_velocity_estimation
{

    double findMedian(std::vector<double> speeds)
    {
        std::sort(speeds.begin(), speeds.end());
        int length = speeds.size();
        double median = 0.0;

        median = (speeds[(length - 1) / 2] + speeds[length / 2]) / 2;

        if (length % 2 != 0)
            median = speeds[length / 2];

        return median;
    }

    RadarPointCloud rejectOutliers(const RadarPointCloud &radar_point_cloud, double range)
    {
        std::vector<double> speeds;
        for (size_t i = 0; i < radar_point_cloud.speed.size(); i++)
        {
            speeds.push_back(-1 * radar_point_cloud.speed[i]);
        }

        double median = findMedian(speeds);

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

        if (radar_point_cloud.points.size() < radar_velocity_settings.min_point_cloud_size)
        {
            return {solution_valid, velocity_estimate, velocity_covariance};
        }

        RadarPointCloud radar_point_cloud_rej = rejectOutliers(radar_point_cloud, radar_velocity_settings.inlier_range_from_median);

        std::vector<gtsam::noiseModel::Robust::shared_ptr> robust_radial_velocity_noise_models;
        auto radial_velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(1, .25);
        robust_radial_velocity_noise_models.push_back(gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Cauchy::Create(3), radial_velocity_noise_model));
        robust_radial_velocity_noise_models.push_back(gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Welsch::Create(3), radial_velocity_noise_model));

        gtsam::Key velocity_key = 0;
        double median = findMedian(radar_point_cloud_rej.speed);
        velocity_estimate = {median, 0, 0};

        for (size_t model = 0; model < robust_radial_velocity_noise_models.size(); model++)
        {
            gtsam::NonlinearFactorGraph factors;
            gtsam::Values states;
            states.insert(velocity_key, velocity_estimate);

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

            if (factors.size() == 0)
            {
                return {solution_valid, velocity_estimate, velocity_covariance};
            }

            try
            {
                gtsam::LevenbergMarquardtOptimizer optimizer(factors, states);
                gtsam::Values solution = optimizer.optimize();

                velocity_estimate = solution.at<gtsam::Vector3>(velocity_key);

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

        double target_x_covariance = pow(.15, 2);
        double covariance_scaling = target_x_covariance / velocity_covariance(0, 0);
        velocity_covariance *= covariance_scaling;

        return {solution_valid, velocity_estimate, velocity_covariance};
    }
}
