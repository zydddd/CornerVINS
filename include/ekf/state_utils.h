/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by Fulin Tang on 2021/1/11.
 */
#ifndef STATEUTILS_H
#define STATEUTILS_H


#include "state.h"
#include "data_types/landmark.h"


namespace Rvg {



/**
 * @brief The class manipulates the State and its covariance.
 */
class StateUtils
{
public:
    /**
     * @brief Performs EKF propagation of the state covariance.
     */
    static void EKFPropagation(std::shared_ptr<State> state, const std::vector<std::shared_ptr<BasicType>> &order_NEW,
            const std::vector<std::shared_ptr<BasicType>> &order_OLD,
                               const Eigen::MatrixXd &Phi, const Eigen::MatrixXd &Q);

    /**
     * @brief Performs EKF update of the state (see @ref linear-meas page)
     */
    static void EKFUpdate(std::shared_ptr<State> state, const std::vector<std::shared_ptr<BasicType>> &H_order,
            const Eigen::MatrixXd &H, const Eigen::VectorXd &res, const Eigen::MatrixXd &R);

    static bool IEKFUpdate(std::shared_ptr<State> state, const std::vector<std::shared_ptr<BasicType>> &H_order,
                           const Eigen::MatrixXd &H, const Eigen::VectorXd &res, const Eigen::MatrixXd &R);

    static void EKFUpdateOri(std::shared_ptr<State> state, const std::vector<std::shared_ptr<BasicType>> &H_order,
                          const Eigen::MatrixXd &H, const Eigen::VectorXd &res, const Eigen::MatrixXd &R);

    static void SynchronizeCornerMap(std::shared_ptr<State> state, const std::vector<int>& toUpdateCornerId);
    static void SynchronizePlaneMap(std::shared_ptr<State> state, const std::vector<int>& toUpdate);
    static void AlignPlaneMap(std::shared_ptr<State> state, std::vector<std::pair<int, int>>& relocate_pairs);
    static void DeletePlaneFromState(std::shared_ptr<State> state, std::shared_ptr<PlaneFeature> plane);

    /**
     * @brief This will fix the initial2 covariance matrix gauge freedoms (4dof, yaw and position).
     */
    static void Fix4dofGaugeFreedoms(std::shared_ptr<State> state, const Eigen::Vector4d &q_GtoI);

    /**
     * @brief For a given set of variables, this will this will calculate a smaller covariance.
     */
    static Eigen::MatrixXd GetMarginalCovariance(std::shared_ptr<State> state,
            const std::vector<std::shared_ptr<BasicType>> &small_variables);

    /**
     * @brief This gets the full covariance matrix.
      */
    static Eigen::MatrixXd GetFullCovariance(std::shared_ptr<State> state);

    /**
     * @brief Marginalizes a variable, properly modifying the ordering/covariances in the state
     */
    static void Marginalize(std::shared_ptr<State> state, std::shared_ptr<BasicType> marg);

    static void MarginalizePlane(std::shared_ptr<State> state);
    static void InitializePlane(std::shared_ptr<State> state, const int map_id);
    static void InitializeCorner(std::shared_ptr<State> state, const int map_id);
    static void InitializeRotation(std::shared_ptr<State> state, const int map_id);
    static void InitializePosition(std::shared_ptr<State> state, const int map_id);
    static void InitializeStructurePlane(std::shared_ptr<State> state, const int map_id);
    /**
     * @brief Clones "variable to clone" and places it at end of covariance
     */
    static std::shared_ptr<BasicType> Clone(std::shared_ptr<State> state, std::shared_ptr<BasicType> variable_to_clone);


    /**
     * @brief Initializes new variable into covariance.
     */
    static bool Initialize(std::shared_ptr<State> state, std::shared_ptr<BasicType> new_variable,
            const std::vector<std::shared_ptr<BasicType>> &H_order, Eigen::MatrixXd &H_R, Eigen::MatrixXd &H_L,
            Eigen::MatrixXd &R, Eigen::VectorXd &res, double chi_2_mult, std::map<int, double>& chi_square);

    static void InitializeVariable(std::shared_ptr<State> state,
                                        std::shared_ptr<BasicType> new_variable,
                                        const Eigen::MatrixXd &R);
    /**
     * @brief Initializes new variable into covariance (H_L must be invertible)
     */
    static void InitializeInvertible(std::shared_ptr<State> state, std::shared_ptr<BasicType> new_variable,
            const std::vector<std::shared_ptr<BasicType>> &H_order, const Eigen::MatrixXd &H_R,
            const Eigen::MatrixXd &H_L, const Eigen::MatrixXd &R, const Eigen::VectorXd &res);


    /**
     * @brief Augment the state with a stochastic copy of the current IMU pose
     */
    static void AugmentClone(std::shared_ptr<State> state, Eigen::Matrix<double, 3, 1> last_w);


    /**
     * @brief Remove the oldest clone, if we have more then the max clone count!!
     */
    static void MarginalizeOldClone(std::shared_ptr<State> state);

    /**
     * @brief Marginalize bad SLAM features
     */
    static void MarginalizeSlam(std::shared_ptr<State> state);


private:
    /**
     * All function in this class should be static.
     * Thus an instance of this class cannot be created.
     */
    StateUtils() {}
};

}




#endif // STATEUTILS_H
