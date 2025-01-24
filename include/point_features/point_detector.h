/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/11.
 */

#ifndef RGBD_PLANE_VIO_POINT_DETECTOR_H
#define RGBD_PLANE_VIO_POINT_DETECTOR_H

#include <vector>
#include <iostream>
#include <functional>
#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



namespace Rvg {


class LambdaBody : public cv::ParallelLoopBody
{
public:
    explicit LambdaBody(const std::function<void(const cv::Range &)> &body)
    {
        body_ = body;
    }
    void operator() (const cv::Range & range) const override
    {
        body_(range);
    }
private:
    std::function<void(const cv::Range &)> body_;
};



/**
 * @brief Extracts FAST features in a small grid.
 */
class PointDetector
{
public:

    /**
     * @brief Compare FAST keypoints based on their response value.
     */
    static bool CompareResponse(cv::KeyPoint first, cv::KeyPoint second)
    {
        return first.response > second.response;
    }


    /**
     * @brief This function will perform grid extraction using FAST.
     * Given a specified grid size, this will try to extract fast features from each grid.
     * It will then return the best from each grid in the return vector.
     */
    static void PerformGriding(const cv::Mat &img, std::vector<cv::KeyPoint> &pts, bool multithread, int num_features,
                                int grid_x, int grid_y, int threshold, bool nonmaxSuppression);
};


}



#endif RGBD_PLANE_VIO_POINT_DETECTOR_H
