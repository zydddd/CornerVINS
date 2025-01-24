/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/19.
 */

#ifndef RGBD_PLANE_VIO_STATIC_INITIALIZER_H
#define RGBD_PLANE_VIO_STATIC_INITIALIZER_H

#include "quat_utils.h"

namespace Rvg {


/**
 * @brief Initializer for visual-inertial system.
 */
    class StaticInitializer
    {
    public:

        /**
         * @brief Struct for a single imu measurement (time, wm, am)
         */
        struct IMUDATA
        {

            /// Timestamp of the reading
            double timestamp;

            /// Gyroscope reading, angular velocity (rad/s)
            Eigen::Matrix<double, 3, 1> wm;

            /// Accelerometer reading, linear acceleration (m/s^2)
            Eigen::Matrix<double, 3, 1> am;

        };

        /**
         * @brief Default constructor
         */
        StaticInitializer(Eigen::Matrix<double,3,1> gravity, double window_length, double imu_excite_threshold) :
                gravity_(gravity), windowLength_(window_length), imuExciteThreshold_(imu_excite_threshold) {}

        /**
         * @brief Stores incoming inertial readings
         */
        void ProcessImu(double timestamp, Eigen::Matrix<double,3,1> wm, Eigen::Matrix<double,3,1> am);


        /**
         * @brief Try to initialize the system using just the imu
         */
        bool InitializeWithImu(double &time0, Eigen::Matrix<double,4,1> &q_GtoI0, Eigen::Matrix<double,3,1> &b_w0,
                               Eigen::Matrix<double,3,1> &v_I0inG, Eigen::Matrix<double,3,1> &b_a0,
                               Eigen::Matrix<double,3,1> &p_I0inG, bool wait_for_jerk=true);


    protected:

        /// Gravity vector in G
        Eigen::Matrix<double,3,1> gravity_;

        /// Amount of time we will initialize over (seconds)
        double windowLength_;

        /// Variance threshold on our acceleration to be classified as moving
        double imuExciteThreshold_;

        /// Our history of IMU messages (time, angular, linear)
        std::vector<IMUDATA> imuData_;


    };
}

#endif //RGBD_PLANE_VIO_STATIC_INITIALIZER_H
