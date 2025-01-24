//
// Created by weihao on 2021/8/7.
//

#ifndef RVG_VIO_VIS_VISUAL_IMU_ALIGNMENT_H
#define RVG_VIO_VIS_VISUAL_IMU_ALIGNMENT_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include "integration_base.h"

namespace Rvg {

class ImageFrame {
public:
    ImageFrame() {};

    ImageFrame(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &_points, double _t)
            : t{_t}, is_key_frame{false} {
        points = _points;
    };
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> points;
    double t;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    IntegrationBase *pre_integration;
    bool is_key_frame;
};

class VisualImuAlignment{
public:

    void solveGyroscopeBias(const int& init_window_size, std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d* Bgs);

    Eigen::MatrixXd TangentBasis(Eigen::Vector3d &g0);

    void RefineGravity(const Eigen::Vector3d& inputG, const std::vector<Eigen::Vector3d>& inputTIC,
                       std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d &g, Eigen::VectorXd &x);

    bool LinearAlignment(const Eigen::Vector3d& inputG, const std::vector<Eigen::Matrix3d>& inputRIC,
                         const std::vector<Eigen::Vector3d>& inputTIC,
                         std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d &g, Eigen::VectorXd &x);


    bool visualImuAlignment(const int& window_size, const Eigen::Vector3d& inputG, const std::vector<Eigen::Matrix3d>& inputRIC,
                            const std::vector<Eigen::Vector3d>& inputTIC, std::map<double, ImageFrame> &all_image_frame, Eigen::Vector3d *Bgs,
                            Eigen::Vector3d &g, Eigen::VectorXd &x);
};


}
#endif //RVG_VIO_VIS_VISUAL_IMU_ALIGNMENT_H
