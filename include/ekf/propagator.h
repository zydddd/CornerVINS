/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by Fulin Tang on 2021/1/11.
 */

#ifndef PROPAGATOR_H
#define PROPAGATOR_H

#include <iomanip>
#include "state_utils.h"
#include "quat_utils.h"


namespace Rvg {


/**
 * @brief Struct for a single imu measurement (time, wm, am)
 */
struct ImuData
{

    /// Timestamp of the reading
    double timestamp;

    /// Gyroscope reading, angular velocity (rad/s)
    Eigen::Matrix<double, 3, 1> wm;

    /// Accelerometer reading, linear acceleration (m/s^2)
    Eigen::Matrix<double, 3, 1> am;

    /// Sort function to allow for using of STL containers
    bool operator<(const ImuData& other) const
    {
        return timestamp < other.timestamp;
    }

};


class Propagator
{
public:

    /**
     * @brief Struct of the imu noise parameters
     */
    struct NoiseParameters
    {

        /// Gyroscope white noise (rad/s/sqrt(hz))
        double sigma_w = 1.6968e-04;

        /// Gyroscope white noise covariance
        double sigma_w_2 = pow(1.6968e-04, 2);

        /// Gyroscope random walk (rad/s^2/sqrt(hz))
        double sigma_wb = 1.9393e-05;

        /// Gyroscope random walk covariance
        double sigma_wb_2 = pow(1.9393e-05, 2);

        /// Accelerometer white noise (m/s^2/sqrt(hz))
        double sigma_a = 2.0000e-3;  // tttt

        /// Accelerometer white noise covariance
        double sigma_a_2 = pow(2.0000e-3, 2);

        /// Accelerometer random walk (m/s^3/sqrt(hz))
        double sigma_ab = 3.0000e-03;

        /// Accelerometer random walk covariance
        double sigma_ab_2 = pow(3.0000e-03, 2);
    };


    /**
     * @brief Default constructor
     */
    Propagator(NoiseParameters noises, Eigen::Vector3d gravity_);


    /**
     * @brief Stores incoming inertial readings
     */
    void ProcessImu(const ImuData &data);


    /**
     * @brief Propagate state up to given timestamp and then clone
     */
    void PropagateAndClone(std::shared_ptr<State> state, double timestamp);


    /**
     * @brief Helper function that given current imu data, will select imu readings between the two times.
     */
    static std::vector<ImuData> SelectImuReadings(const std::vector<ImuData>& imuData_, double time0, double time1);


    /**
     * @brief Nice helper function that will linearly interpolate between two imu messages.
     */
    static ImuData InterpolateData(const ImuData &imu_1, const ImuData &imu_2, double timestamp);


    std::vector<ImuData> imuDataCurr_;
    double time_diff;

    std::map<double, Eigen::Matrix<double,3,3>> Rot_GtoI_gyro_frame;  // imu时间戳，世界坐标系到这一帧相机坐标系的旋转矩阵
    std::map<double, Eigen::Matrix<double,7,1>> Rot_and_trans_GtoI_gyro_frame;

protected:


    /**
     * @brief Propagates the state forward using the imu data and computes the noise covariance and state-transition
     * matrix of this interval.
     */
    void PredictAndCompute(std::shared_ptr<State> state, const ImuData &data_minus, const ImuData &data_plus,
            Eigen::Matrix<double, 15, 15> &F, Eigen::Matrix<double, 15, 15> &Qd);


    /**
     * @brief Discrete imu mean propagation.
     */
    void PredictMeanDiscrete(std::shared_ptr<State> state, double dt,
                                 const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                                 const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2,
                                 Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p);



    /**
     * @brief RK4 imu mean propagation.
     */
    void PredictMeanRk4(std::shared_ptr<State> state, double dt,
                          const Eigen::Vector3d &w_hat1, const Eigen::Vector3d &a_hat1,
                          const Eigen::Vector3d &w_hat2, const Eigen::Vector3d &a_hat2,
                          Eigen::Vector4d &new_q, Eigen::Vector3d &new_v, Eigen::Vector3d &new_p);


    /// Container for the noise values
    NoiseParameters noisesParameters_;

    /// Our history of IMU messages (time, angular, linear)
    std::vector<ImuData> imuData_;

    /// Gravity vector
    Eigen::Matrix<double, 3, 1> gravity_;

    /// Estimate for time offset (camera and imu) at last propagation time
    double lastPropTimeOffset_ = 0.0;
    bool haveLastPropTimeOffset_ = false;

};


}


#endif // PROPAGATOR_H
