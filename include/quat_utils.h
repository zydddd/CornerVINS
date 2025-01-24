//
// Created by zyd on 23-8-10.
//

#ifndef RGBD_PLANE_VIO_QUAT_UTILS_H
#define RGBD_PLANE_VIO_QUAT_UTILS_H

#include <string>
#include <sstream>
#include <iostream>
#include <Eigen/Eigen>

namespace Rvg
{

/**
 * @brief this class contains the common utility functions for operating on JPL quaternions.
 */
    class QuatUtils
    {

    public:

        /**
         * @brief Returns a JPL quaternion from a rotation matrix
         */
        static Eigen::Matrix<double, 4, 1> Rot2Quat(const Eigen::Matrix<double, 3, 3> &rot);


        /**
         * @brief Skew-symmetric matrix from a given 3x1 vector
         */
        static Eigen::Matrix<double, 3, 3> SkewX(const Eigen::Matrix<double, 3, 1> &w);


        /**
         * @brief Converts JPL quaterion to SO(3) rotation matrix
         */
        static Eigen::Matrix<double, 3, 3> Quat2Rot(const Eigen::Matrix<double, 4, 1> &q);


        /**
         * @brief Multiply two JPL quaternions
         */
        static Eigen::Matrix<double, 4, 1> QuatMultiply(const Eigen::Matrix<double, 4, 1> &q, const Eigen::Matrix<double, 4, 1> &p);


        /**
         * @brief Returns vector portion of skew-symmetric
         */
        static Eigen::Matrix<double, 3, 1> Vee(const Eigen::Matrix<double, 3, 3> &w_x);


        /**
         * @brief SO(3) matrix exponential
         */
        static Eigen::Matrix<double, 3, 3> ExpSo3(const Eigen::Matrix<double, 3, 1> &w);


        /**
         * @brief SO(3) matrix logarithm
         */
        static Eigen::Matrix<double, 3, 1> LogSo3(const Eigen::Matrix<double, 3, 3> &R);


        /**
         * @brief SE(3) matrix exponential function
         */
        static Eigen::Matrix4d ExpSe3(Eigen::Matrix<double,6,1> vec);


        /**
         * @brief SE(3) matrix logarithm
         */
        static Eigen::Matrix<double,6,1> LogSe3(Eigen::Matrix4d mat);


        /**
         * @brief Hat operator for R^6 -> Lie Algebra se(3)
         */
        static Eigen::Matrix4d HatSe3(const Eigen::Matrix<double,6,1> &vec);


        /**
         * @brief SE(3) matrix analytical inverse
         */
        static Eigen::Matrix4d InvSe3(const Eigen::Matrix4d &T);


        /**
         * @brief JPL Quaternion inverse
         */
        static Eigen::Matrix<double, 4, 1> Inv(Eigen::Matrix<double, 4, 1> q);


        /**
         * @brief Integrated quaternion from angular velocity
         *
         */
        static Eigen::Matrix<double, 4, 4> Omega(Eigen::Matrix<double, 3, 1> w);


        /**
         * @brief Normalizes a quaternion to make sure it is unit norm
         */
        static Eigen::Matrix<double, 4, 1> QuatNorm(Eigen::Matrix<double, 4, 1> q_t);


        /**
         * @brief Computes left Jacobian of SO(3)
         */
        static Eigen::Matrix<double, 3, 3> JlSo3(Eigen::Matrix<double, 3, 1> w);


        /**
         * @brief Computes right Jacobian of SO(3)
         */
        static Eigen::Matrix<double, 3, 3> JrSo3(Eigen::Matrix<double, 3, 1> w);


        static Eigen::Matrix<double, 4, 4> QlMatrix(Eigen::Matrix<double, 4, 1> q_t);
        static Eigen::Matrix<double, 4, 4> QrMatrix(Eigen::Matrix<double, 4, 1> q_t);

    };


}

#endif //RGBD_PLANE_VIO_QUAT_UTILS_H
