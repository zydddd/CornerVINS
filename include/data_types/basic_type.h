/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by Fulin Tang on 2021/1/11.
 */

#ifndef BASICTYPE_H
#define BASICTYPE_H

#include <memory>
#include <Eigen/Eigen>


namespace Rvg {

enum DataType
{
    BASICTYPE =0,
    POSEJPL = 1,
    LANDMARK = 2,
    IMUTYPE = 3
};

/**
 * @brief Base class for estimated variables.
 *
 * This class is used how variables are represented or updated (e.g., vectors or quaternions).
 * Each variable is defined by its error state size and its location in the covariance matrix.
 * We additionally require all sub-types to have a update procedure.
 */
class BasicType
{
public:

    /**
     * @brief Default constructor for basic type
     *
     * @param size_ degrees of freedom of variable (i.e., the size of the error state)
     */
    BasicType(int size_);

    virtual ~BasicType();

    /**
     * @brief Sets id used to track location of variable in the filter covariance
     *
     * Note that the minimum ID is -1 which says that the state is not in our covariance.
     * If the ID is larger than -1 then this is the index location in the covariance matrix.
     *
     * @param new_id entry in filter covariance corresponding to this variable
     */
    virtual void SetLocalId(int new_id);

    /**
     * @brief Access to variable id (i.e. its location in the covariance)
     */
    int GetId();

    /**
     * @brief Access to variable size (i.e. its error state size)
     */
    int GetSize();

    /**
     * @brief Update variable due to perturbation of error state
     *
     * @param dx Perturbation used to update the variable through a defined "boxplus" operation
     */
    virtual void Update(const Eigen::VectorXd& dx) = 0;

    /**
     * @brief Access variable's estimate
     */
    virtual const Eigen::MatrixXd& GetValue() const;

    /**
     * @brief Access variable's first-estimate
     */
    virtual const Eigen::MatrixXd& GetFej() const;

    /**
     * @brief Overwrite value of state's estimate
     * @param new_value New value that will overwrite state's value
     */
    virtual void SetValue(const Eigen::MatrixXd& new_value);

    /**
     * @brief Overwrite value of first-estimate
     * @param new_value New value that will overwrite state's fej
     */
    virtual void SetFej(const Eigen::MatrixXd& new_value);

    /**
     * @brief Create a clone of this variable
     */
    virtual std::shared_ptr<BasicType> Clone() = 0;

    /**
     * @brief Determine if pass variable is a sub-variable
     *
     * If the passed variable is a sub-variable or the current variable this will return it.
     * Otherwise it will return a nullptr, meaning that it was unable to be found.
     *
     * @param check Type pointer to compare our subvariables to
     */
    virtual std::shared_ptr<BasicType> CheckIfSubvariable(const std::shared_ptr<BasicType> check);


    /**
     * @brief class type trails
     */
    virtual DataType ClassofSubvariable();


protected:

    /// First-estimate matrix: estimate from the first time
    Eigen::MatrixXd fej;

    /// Current best estimation matrix
    Eigen::MatrixXd value;

    /// Location of error state in covariance matrix
    int id = -1;

    /// Dimension of error state
    int size = -1;

    bool should_marg;

};


}



#endif // BASICTYPE_H
