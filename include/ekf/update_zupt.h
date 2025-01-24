//
// Created by weihao on 2022/6/3.
//

#ifndef RGBD_PLANE_VIO_UPDATE_ZUPT_H
#define RGBD_PLANE_VIO_UPDATE_ZUPT_H

#include "quat_utils.h"
#include "ekf/state.h"
#include "ekf/state_utils.h"
#include "ekf/update_utility.h"
#include "propagator.h"

#include <chrono>
#include <random>

namespace Rvg {

/**
 * @brief Zero Velocity Detection and Update
 * */
class UpdateZUPT
{
public:

    /**
     * @brief Default constructor for our zero velocity detector and updater.
     * @param options Updater options (chi2 multiplier)
     * @param noises imu noise characteristics (continuous time)
     * @param gravity Global gravity of the system (normally [0,0,9.81])
     * @param zupt_max_velocity Max velocity we should consider to do a update with
     * @param zupt_noise_multiplier Multiplier of our IMU noise matrix (default should be 1.0)
     */
    UpdateZUPT(UpdaterParameters &options, Propagator::NoiseParameters &noises, std::map<int, double>& chi_square, Eigen::Vector3d gravity, double zupt_max_velocity, double zupt_noise_multiplier)
    : parameters_(options), noises_(noises), chiSquaredTable_(chi_square), gravity_(gravity), zupt_max_velocity_(zupt_max_velocity), zupt_noise_multiplier_(zupt_noise_multiplier) {
        parameters_.sigma_pix_sq = std::pow(parameters_.sigma_pix, 2);
    }


    /**
     * @brief Stores incoming inertial readings
     */
    void FeedImu(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am) {

        // Create our imu data object
        ImuData data;
        data.timestamp = timestamp;
        data.wm = wm;
        data.am = am;

        // Append it to our vector
        imuData_.emplace_back(data);

        // Loop through and delete imu messages that are older then 60 seconds
        auto it0 = imuData_.begin();
        while(it0 != imuData_.end()) {
            if(timestamp-(*it0).timestamp > 60) {
                it0 = imuData_.erase(it0);
            } else {
                it0++;
            }
        }
    }


    /**
     * @brief Will first detect if the system is zero velocity, then will update.
     */
    bool TryUpdate(std::shared_ptr<State> state, double timestamp);



protected:
    /// Options used during update (chi2 multiplier)
    UpdaterParameters parameters_;

    /// Container for the imu noise values
    Propagator::NoiseParameters noises_;

    /// Gravity vector
    Eigen::Matrix<double, 3, 1> gravity_;

    /// Max velocity (m/s) that we should consider a zupt with
    double zupt_max_velocity_ = 1.0;

    /// Multiplier of our IMU noise matrix (default should be 1.0)
    double zupt_noise_multiplier_ = 1.0;

    /// Chi squared 95th percentile table (lookup would be size of residual)
    std::map<int, double> chiSquaredTable_;

    /// Our history of IMU messages (time, angular, linear)
    std::vector<ImuData> imuData_;

    /// Estimate for time offset at last propagation time
    double last_prop_time_offset = -INFINITY;
    bool have_last_prop_time_offset = false;
};

}
#endif RGBD_PLANE_VIO_UPDATE_ZUPT_H
