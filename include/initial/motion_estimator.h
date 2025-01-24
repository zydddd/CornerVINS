//
// Created by weihao on 2021/8/7.
//
#ifndef RVG_VIO_VIS_MOTION_ESTIMATOR_H
#define RVG_VIO_VIS_MOTION_ESTIMATOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

namespace Rvg {

class MotionEstimator {
public:

    /**
     * @brief Use five-point-algorithm to compute relative pose (R and T)
     * @param corres corresponding point feature in normalized frame
     * @param R rotation matrix
     * @param T translation matrix
     * */
    bool solveRelativeRT(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &corres,
                         Eigen::Matrix3d &R, Eigen::Vector3d &T);

private:
    double testTriangulation(const std::vector <cv::Point2f> &l, const std::vector <cv::Point2f> &r,
                             cv::Mat_<double> R, cv::Mat_<double> t);

    void decomposeE(cv::Mat E, cv::Mat_<double> &R1, cv::Mat_<double> &R2, cv::Mat_<double> &t1, cv::Mat_<double> &t2);
};
}
#endif //RVG_VIO_VIS_MOTION_ESTIMATOR_H
