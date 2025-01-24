/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by Fulin Tang on 2021/1/11.
 */

#ifndef IMU_H
#define IMU_H

#include "pose_jpl.h"
#include "quat_utils.h"

namespace Rvg {

/**
 * @brief The class implements an IMU state
 */
class IMU : public BasicType
{

public:
    IMU();

    ~IMU();


    /**
     * @brief Sets id used to track location of variable in the filter covariance
     * Note that we update the sub-variables also.
     * @param new_id entry in filter covariance corresponding to this variable
     */
    void SetLocalId(int new_id) override;


    /**
      * @brief Performs update operation using JPLQuat update for orientation, then vector updates for
      * position, velocity, gyro bias, and accel bias (in that order).
      * @param dx 15 DOF vector encoding update using the following order (q, p, v, bg, ba)
      */
    void Update(const Eigen::VectorXd& dx) override;

    DataType ClassofSubvariable() override;


    /**
     * @brief Sets the value of the estimate
     * @param new_value New value we should set
     */
    void SetValue(const Eigen::MatrixXd& new_value) override;


    /**
     * @brief Sets the value of the first estimate
     * @param new_value New value we should set
     */
    void SetFej(const Eigen::MatrixXd& new_value) override;

    std::shared_ptr<BasicType> Clone() override;

    std::shared_ptr<BasicType> CheckIfSubvariable(const std::shared_ptr<BasicType> check) override;

    /// Rotation access
    Eigen::Matrix<double, 3, 3> GetRotation() const;
    /// FEJ Rotation access
    Eigen::Matrix<double, 3, 3> GetRotationFej() const;
    /// Rotation access quaternion
    Eigen::Matrix<double, 4, 1> GetQuat() const;
    /// FEJ Rotation access quaternion
    Eigen::Matrix<double, 4, 1> GetQuatFej() const;
    /// Position access
    Eigen::Matrix<double, 3, 1> GetPos() const;
    /// FEJ position access
    Eigen::Matrix<double, 3, 1> GetPosFej() const;
    /// Velocity access
    Eigen::Matrix<double, 3, 1> GetVel() const;
    // FEJ velocity access
    Eigen::Matrix<double, 3, 1> GetVelFej() const;
    /// Gyro bias access
    Eigen::Matrix<double, 3, 1> GetBiasG() const;
    /// FEJ gyro bias access
    Eigen::Matrix<double, 3, 1> GetBiasGFej() const;
    /// Accel bias access
    Eigen::Matrix<double, 3, 1> GetBiasA() const;
    // FEJ accel bias access
    Eigen::Matrix<double, 3, 1> GetBiasAFej() const;
    /// Pose type access
    std::shared_ptr<PoseJPL> GetPose();
    /// Quaternion type access
    std::shared_ptr<JPLQuat> GetQ();
    /// Position type access
    std::shared_ptr<Vec> GetP();
    /// Velocity type access
    std::shared_ptr<Vec> GetV();
    /// Gyroscope bias access
    std::shared_ptr<Vec> GetBg();
    /// Acceleration bias access
    std::shared_ptr<Vec> GetBa();

protected:

    /// Pose subvariable
    std::shared_ptr<PoseJPL> pose;

    /// Velocity subvariable
    std::shared_ptr<Vec> v;

    /// Gyroscope bias subvariable
    std::shared_ptr<Vec> bg;

    /// Acceleration bias subvariable
    std::shared_ptr<Vec> ba;
};


}




#endif // IMU_H
