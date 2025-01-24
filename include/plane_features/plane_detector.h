//
// Created by zyd on 23-12-14.
//

#ifndef RGBD_PLANE_VIO_PLANE_DECTOR_H
#define RGBD_PLANE_VIO_PLANE_DECTOR_H


#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <iostream>
#include <unordered_map>
#include <Eigen/Eigen>

#include "peac/AHCPlaneFitter.hpp"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

#include "plane_feature.h"
#include "quat_utils.h"
#include "utils.h"

#include <mutex>

namespace Rvg {

    struct ImagePointCloud
    {
        std::vector<Eigen::Vector3d> vertices; // 3D vertices
        int w, h;

        inline int width() const { return w; }
        inline int height() const { return h; }
        inline bool get(const int row, const int col, double &x, double &y, double &z) const {
            const int pixIdx = row * w + col;
            z = vertices[pixIdx][2];
            // Remove points with 0 or invalid depth in case they are detected as a plane
            if (z == 0 || std::isnan(z)) return false;
            x = vertices[pixIdx][0];
            y = vertices[pixIdx][1];
            return true;
        }
    };


    class PlaneDetector
    {
    public:

        std::vector<int> n_sp;  //
        std::vector<int> n_ds;  //
        std::vector<int> n_fit;  //
        std::vector<int> n_ds_merge;
        std::vector<int> n_refit;

        PlaneDetector(Eigen::VectorXd camera_calib, double kScaleFactor, double angle, double dis, double n);
        ~PlaneDetector();

        bool ReadColorImage(cv::Mat& RGBImg);
        bool ReadDepthImage(cv::Mat& depthImg);

        void AHCPlaneSegmentation();  //
        void AppendNewPlanes(double timestamp);
        void ComputeNoise();

        bool MaxPointDistanceFromPlane(cv::Mat &plane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, pcl::PointIndices::Ptr inliers);
        bool IsMatch(const std::shared_ptr<PlaneSegmentCoeff>& p1, const std::shared_ptr<PlaneSegmentCoeff>& p2);


        void DisplayActive(cv::Mat& img);


        ////////////////
        void RunBorderDetection(double timestamp);
        bool ComputeIntersectionLine(std::pair<Eigen::Vector3d, Eigen::Vector3d>& endPoints,
                                     const std::shared_ptr<PlaneCoeff>& p1, const std::shared_ptr<PlaneCoeff>& p2);
        bool ComputeIntersectionLine(std::pair<Eigen::Vector3d, Eigen::Vector3d>& endPoints,
                                     const std::shared_ptr<PlaneSegmentCoeff>& p1, const std::shared_ptr<PlaneSegmentCoeff>& p2);

        ImagePointCloud cloud;
        ahc::PlaneFitter<ImagePointCloud> plane_filter;
        std::vector<std::vector<int>> plane_vertices_; // vertex indices each plane contains
        cv::Mat seg_img_; // segmentation image
        cv::Mat color_img_; // input color image
        cv::Mat depth_img_; // input depth image

        std::vector<std::shared_ptr<PlaneObservation>> plane_detected;
        std::vector<std::shared_ptr<BorderObservation>> border_detected;
        std::vector<std::shared_ptr<CornerObservation>> corner_detected;

    private:
        cv::Mat K_;
        double kScaleFactor_ = 1 / 1000.0;

        const unsigned char colors_bgr[80][3] =
        {
                {113, 154, 172}, {138, 140, 191}, {184, 168, 207}, {231, 188, 198},
                {253, 207, 158}, {239, 164, 132}, {182, 118, 108}, {128, 116, 200},
                {120, 149, 193}, {168, 203, 223}, {153, 34, 36}, {214, 239, 244},
                {181, 71, 100}, {227, 98, 93}, {239, 139, 103}, {240, 194, 132},
                {245, 235, 174}, {157, 158, 163}, {183, 183, 235}, {155, 187, 225},
                {240, 155, 160}, {0, 138, 69}, {255, 211, 115}, {242, 120, 155},
                {128, 197, 162}, {95, 95, 94}, {70, 139, 202}, {179, 132, 186},
                {191, 223, 210}, {237, 140, 90}, {104, 190, 217}, {37, 125, 139},
                {78, 101, 155}, {114, 176, 99}, {226, 145, 53}, {74, 95, 126}, //
                {60, 118, 110}, {111, 122, 146}, {148, 141, 162}, {186, 161, 154},
                {58, 120, 105}, {108, 120, 141}, {144, 138, 157}, {181, 158, 149},
                {56, 122, 100}, {105, 118, 136}, {140, 135, 152}, {176, 155, 144},
                {54, 124, 95}, {102, 116, 131}, {136, 132, 147}, {171, 152, 139},
                {52, 126, 90}, {99, 114, 126}, {132, 129, 142}, {166, 149, 134},
                {50, 128, 85}, {96, 112, 121}, {128, 126, 137}, {161, 146, 129},
                {48, 130, 80}, {93, 110, 116}, {124, 123, 132}, {156, 143, 124},
                {46, 132, 75}, {90, 108, 111}, {120, 120, 127}, {151, 140, 119},
                {44, 134, 70}, {87, 106, 106}, {116, 117, 122}, {146, 137, 114},
                {42, 136, 65}, {84, 104, 101}, {112, 114, 117}, {141, 134, 109},
                {40, 138, 60}, {81, 102, 96}, {108, 111, 112}, {136, 131, 104}
        };

        //// 以下参数由构造函数从配置文件读取
        double angle_thres = 5;  // for merging two segments
        double dis_thres = 0.02;  // for merging two segments, for refitting
        double noise = 0;

        //// 以下参数需要手动修改，没有从配置文件传
        double valid_depth_near_ = 0.1;
        double valid_depth_far_ = 5.0;
        int size_thres = 20000;  //
        bool refine_ahc_detect = true;  //
        bool merge_segments = true;
        double inlier_rate_ = 0.9; //
        double downsample_leaf_size = 0.05;
        bool refine_segmentation_ = true;



        //// Corner相关
        double edge_legnth_ = 0.30; // l
        double endpoints_intersection_dis_ = 0.3;  // 交
        double intersection_d_thres_ = 0.1;  //
        // 以下参数需要手动修改，没有从配置文件传
        bool refine_plane_params_ = true;  //
        double vertical_rad_thres_ = 85 * M_PI / 180.0;  //


    };

}
#endif //RGBD_PLANE_VIO_PLANE_DECTOR_H