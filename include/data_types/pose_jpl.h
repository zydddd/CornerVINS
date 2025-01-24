/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by Fulin Tang on 2021/1/11.
 */

#ifndef POSEJPL_H
#define POSEJPL_H

#include "vec.h"
#include "jpl_quat.h"
#include "quat_utils.h"


namespace Rvg {


/**
 * @brief The class implements a 6 d.o.f pose
 */
class PoseJPL : public BasicType
{
public:
    PoseJPL();

     ~PoseJPL();


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



#endif // POSEJPL_H
