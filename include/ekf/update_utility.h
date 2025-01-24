/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/12.
 */

#ifndef RGBD_PLANE_VIO_UPDATE_UTILITY_H
#define RGBD_PLANE_VIO_UPDATE_UTILITY_H

#include <Eigen/Eigen>

#include "point_features/point_feature.h"
#include "data_types/landmark.h"
#include "ekf/state.h"
#include "quat_utils.h"

namespace Rvg {


/**
 * @brief Struct which stores general updater options
 */
struct UpdaterParameters
{
    /// What chi-squared multipler we should apply
    int chi2_multipler = 5;
    /// Noise sigma for our raw pixel measurements
    double sigma_pix = 1;
    /// Covariance for our raw pixel measurements
    double sigma_pix_sq = 1;

    bool use_mad = false;
    bool with_depth = false;
};

/**
 * @brief Class that has helper functions for our updaters.
 */
class UpdateUtility
{

public:

    /**
     * @brief Feature object that our UpdateUtility leverages, has all measurements and means
     */
    struct UpdateUtilityFeature
    {
        /// Unique ID of this feature
        size_t feat_id;

        /// UV coordinates that this feature has been seen from (mapped by camera ID)
        std::vector<Eigen::VectorXf> uvs;
        // UV normalized coordinates that this feature has been seen from (mapped by camera ID)
        std::vector<Eigen::VectorXf> uvs_norm;
        /// Timestamps of each UV measurement (mapped by camera ID)
        std::vector<double> timestamps;
        std::vector<double> depths;

        /// What representation our feature is in
        LandmarkRepresentation::Representation feat_representation;

        /// Timestamp of anchor clone
        double anchor_clone_timestamp = -1;

        /// Triangulated position of this feature, in the anchor frame
        Eigen::Vector3d p_FinA;
        /// Triangulated position of this feature, in the anchor frame first estimate
        Eigen::Vector3d p_FinA_fej;
        /// Triangulated position of this feature, in the global frame
        Eigen::Vector3d p_FinG;
        /// Triangulated position of this feature, in the global frame first estimate
        Eigen::Vector3d p_FinG_fej;
    };


    /**
     * @brief This gets the feature and state Jacobian in respect to the feature representation
     */
    static void GetFeatureJacobianRepresentation(std::shared_ptr<State> state, UpdateUtilityFeature &feature,
            Eigen::MatrixXd &H_f, std::vector<Eigen::MatrixXd> &H_x, std::vector<std::shared_ptr<BasicType>> &x_order);
    static void GetFeatureJacobianRepresentationInit(std::shared_ptr<State> state, UpdateUtilityFeature &feature,
                                                 Eigen::MatrixXd &H_f, std::vector<Eigen::MatrixXd> &H_x, std::vector<std::shared_ptr<BasicType>> &x_order);
    static void GetFeatureJacobianFullWithDepthTest(std::shared_ptr<State> state,
                                                            UpdateUtilityFeature &feature,
                                                            Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                                                            Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order, Eigen::VectorXd &test);
    /**
     * @brief This will compute the Jacobian in respect to the intrinsic calibration parameters and normalized coordinates
     */
    static void GetFeatureJacobianIntrinsics(std::shared_ptr<State> state, const Eigen::Vector2d &uv_norm,
            bool isfisheye, Eigen::Matrix<double,8,1> cam_d, Eigen::Matrix<double,2,2> &dz_dzn,
            Eigen::Matrix<double,2,8> &dz_dzeta);


    /**
     * @brief Will construct the "stacked" Jacobians for a single feature from all its measurements
     */
    static void GetFeatureJacobianFull(std::shared_ptr<State> state, UpdateUtilityFeature &feature,
            Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res,
            std::vector<std::shared_ptr<BasicType>> &x_order);

    static void GetFeatureJacobianFullWithDepth(std::shared_ptr<State> state,
                                        UpdateUtilityFeature &feature,
                                        Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                                        Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order);
    /**
     * @brief This will project the left nullspace of H_f onto the linear system.
     */
    static void NullspaceProjectInplace(Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x, Eigen::VectorXd &res);


    /**
     * @brief This will perform measurement compression
     */
    static void MeasurementCompressInplace(Eigen::MatrixXd &H_x, Eigen::VectorXd &res);
    static void MeasurementCompressInplace(Eigen::MatrixXd &H_x,
                                                   Eigen::VectorXd &res,
                                                   Eigen::MatrixXd &R);

};
}

#endif RGBD_PLANE_VIO_UPDATE_UTILITY_H
