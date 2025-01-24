//
// Created by zyd on 23-1-2.
//

#ifndef RGBD_PLANE_VIO_UPDATE_PLANE_H
#define RGBD_PLANE_VIO_UPDATE_PLANE_H

#include <Eigen/Eigen>

#include "point_features/point_feature.h"
#include "point_features/point_triangulation.h"
#include "plane_feature.h"

#include "data_types/landmark.h"
#include "quat_utils.h"
#include "ekf/state.h"
#include "ekf/state_utils.h"
#include "ekf/update_utility.h"

#include <chrono>
#include <random>
#include <memory>

namespace Rvg {

class UpdatePlane
{
public:
    UpdatePlane(double noise, std::map<int, double>& chi_square, int chi2_multipler, int up_meas);

    void Update(std::shared_ptr<State> state, std::vector<std::shared_ptr<PlaneObservation>>& feature_vec);
    void UpdateAngles(std::shared_ptr<State> state, std::vector<std::shared_ptr<PlaneObservation>>& feature_vec);
    void UpdateCorners(std::shared_ptr<State> state, std::vector<std::shared_ptr<CornerObservation>>& feature_vec);
    void DeDriftUpdate(std::shared_ptr<State> state, const std::vector<std::pair<int,int>>& relocate_pair);
    void CoPlanarUpdate(std::shared_ptr<State> state, const std::vector<std::pair<int, std::vector<int>>> ids);
    void GetFreePlaneJacobianFull(std::shared_ptr<State> state,
                                std::shared_ptr<PlaneFeature> feature, std::vector<int>& index,
                                Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                                Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order, Eigen::MatrixXd &R);
    void GetFreePlaneJacobianP(std::shared_ptr<State> state,
                                  std::shared_ptr<PlaneFeature> feature, std::vector<int>& index,
                                  Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                                  Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order, Eigen::MatrixXd &R);

    void GetStructurePlaneJacobianFull(std::shared_ptr<State> state,
                                  std::shared_ptr<PlaneFeature> feature, std::vector<int>& index,
                                  Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                                  Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order, Eigen::MatrixXd &R);
    void GetCoPlanarFreeJacobianFull(std::shared_ptr<State> state,
                                       std::shared_ptr<PlaneFeature> feature, std::vector<int>& index,
                                       Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                                       Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order, Eigen::MatrixXd &R);
    void GetCoPlanarStructureJacobianFull(std::shared_ptr<State> state,
                                     std::shared_ptr<PlaneFeature> feature, std::vector<int>& index,
                                     Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                                     Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order, Eigen::MatrixXd &R);

    void GetStructurePlaneJacobianP(std::shared_ptr<State> state,
                                       std::shared_ptr<PlaneFeature> feature, std::vector<int>& index,
                                       Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                                       Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order, Eigen::MatrixXd &R);

    void GetCornerJacobianFull(std::shared_ptr<State> state,
                               std::shared_ptr<CornerFeature> corner_feature, std::vector<int>& obs_index,
                               Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                               Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order, Eigen::MatrixXd& R);

    bool this_use_corner_update_ = false;
protected:
    std::ofstream debug_file;
    double planeNoise_ = 1;

    std::map<int, double> chiSquaredTable_;

    /// What chi-squared multipler we should apply
    int chi2_multipler_ = 1;

    int min_measurements_ = 2;


    /// 以下参数需要手动修改，未从配置文件传参
    bool reuse_corner_plane = true;  //
    double coplanarNoise_ = 0.001;
    double corner_min_measurements_ = 3;
    double corner_rotation_noise = 0.05;
    double corner_position_noise = 1e-2;

};

}


#endif RGBD_PLANE_VIO_UPDATE_PLANE_H
