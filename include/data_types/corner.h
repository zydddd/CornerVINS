//
// Created by zyd on 24-6-13.
//

#ifndef RGBD_PLANE_VIO_CORNER_H
#define RGBD_PLANE_VIO_CORNER_H

#include "vec.h"
#include "jpl_quat.h"
#include <iostream>

namespace Rvg {

    class Position: public Vec
    {
    public:
        Position(int dim);
        void Update(const Eigen::VectorXd& dx) override;
        bool should_marg = false;
    };

    class Rotation: public JPLQuat
    {
    public:
        Rotation();
        ~Rotation();
        bool should_marg = false;
    };

    class Corner: public BasicType
    {
    public:
        Corner();

        ~Corner();
/**
     * @brief Sets id used to track location of variable in the filter covariance
     */
        void SetLocalId(int new_id) override;


        /**
         * @brief Update q and p using a the JPLQuat update for orientation and vector update for position
         */
        void Update(const Eigen::VectorXd& dx) override;


        /**
         * @brief Sets the value of the estimate
         */
        void SetValue(const Eigen::MatrixXd& new_value) override;


        /**
         * @brief Sets the value of the first estimate
         */
        void SetFej(const Eigen::MatrixXd& new_value) override;

        std::shared_ptr<BasicType> Clone() override;

        std::shared_ptr<BasicType> CheckIfSubvariable(const std::shared_ptr<BasicType> check) override;

        DataType ClassofSubvariable() override;

        /// Rotation access
        Eigen::Matrix<double, 3, 3> GetRotation() const;
        /// FEJ Rotation access
        Eigen::Matrix<double, 3, 3> GetRotationFej() const;
        /// Rotation access as quaternion
        Eigen::Matrix<double, 4, 1> GetQuat() const;
        /// FEJ Rotation access as quaternion
        Eigen::Matrix<double, 4, 1> GetQuatFej() const;
        /// Position access
        Eigen::Matrix<double, 3, 1> GetPos() const;
        // FEJ position access
        Eigen::Matrix<double, 3, 1> GetPosFej() const;
        // Quaternion type access
        std::shared_ptr<JPLQuat> GetQ();
        // Position type access
        std::shared_ptr<Vec> GetP();

    protected:
        /// Subvariable containing orientation
        std::shared_ptr<JPLQuat> q;

        /// Subvariable containing position
        std::shared_ptr<Vec> p;
    };
}

#endif //RGBD_PLANE_VIO_CORNER_H
