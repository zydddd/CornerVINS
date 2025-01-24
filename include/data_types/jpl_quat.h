/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by Fulin Tang on 2021/1/11.
 */

#ifndef JPLQUAT_H
#define JPLQUAT_H


#include "basic_type.h"
#include "quat_utils.h"



namespace Rvg {


/**
 * @brief The class implements JPL quaternion based on BasicType
 * This quaternion uses a left-multiplicative error state and follows the "JPL convention".
 */
class JPLQuat : public BasicType
{
public:
    JPLQuat();

    ~JPLQuat();

    /**
     * @brief Implements update operation by left-multiplying the current
     */
    void Update(const Eigen::VectorXd& dx) override;

    /**
     * @brief Sets the value of the estimate and recomputes the internal rotation matrix
     * @param new_value New value for the quaternion estimate
     */
    void SetValue(const Eigen::MatrixXd& new_value) override;

    /**
     * @brief Sets the fej value and recomputes the fej rotation matrix
     * @param new_value New value for the quaternion estimate
     */
    void SetFej(const Eigen::MatrixXd& new_value) override;

    std::shared_ptr<BasicType> Clone() override;

    /// Rotation access
    Eigen::Matrix<double, 3, 3> GetRotation() const;

    /// FEJ Rotation access
    Eigen::Matrix<double, 3, 3> GetRotationFej() const;


protected:

    // Stores the rotation
    Eigen::Matrix<double, 3, 3> R;

    // Stores the first-estimate rotation
    Eigen::Matrix<double, 3, 3> Rfej;

};


}



#endif // JPLQUAT_H
