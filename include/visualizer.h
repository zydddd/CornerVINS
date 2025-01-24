/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/13.
 */

#ifndef RGBD_PLANE_VIO_VISUALIZER_H
#define RGBD_PLANE_VIO_VISUALIZER_H

#include "vio_system.h"
#include "dataset_reader.h"

#include <pangolin/pangolin.h>
#include <mutex>

namespace Rvg
{


/**
 * @brief Visualizer for publish results.
 *
 * We visualize the following things:
 * - State of the system, pose message, and path
 * - Image of our tracker
 */



class Visualizer
{
public:
    /**
     * @brief Default constructor
     * @param app Core estimator manager
     * @param filePath config file path
     */
    Visualizer();
    Visualizer(std::shared_ptr<VIOSystem> app, std::string filePath, bool auto_exit);

    ~Visualizer();


    /**
     * @brief Will visualize the system if we have new things
     */
    void Visualize();
    void VisualizeWithGt();
    /**
     * @brief After the run has ended, print final results
     */
    void VisualizeFinal();

    /**
     * @brief Draw pose and trajectory use pangolin
     */
    void DrawTrajectory();
    void DrawDelaunay();
    void DrawCameraViewMap();
    std::vector<Eigen::Vector3d> CalculateCubeVertices(Eigen::Vector3d point, Eigen::Vector3d dir1, Eigen::Vector3d dir2, Eigen::Vector3d dir3);
protected:

    /// Publish the current state
    void PublishState();
    void PublishGtState();
    /// Publish the feature tracking image
    void PublishImages();

    cv::Mat GetCurrentImageDebugInformation();

    void AddTextToImage(std::string &s, cv::Mat &im, const int r, const int g, const int b);

    /// Core application of the filter system
    std::shared_ptr<VIOSystem> app_;


    // vector to store path
    std::vector<Eigen::Matrix3d> rotationMatrix_;
    std::vector<Eigen::Vector3d> translationMatrix_;

    std::vector<Eigen::Vector3d> msckfClouds_;
    std::vector<Eigen::Vector3d> slamClouds_;
    std::vector<Eigen::Vector3d> initClouds_;
    std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> planeClouds_;
    std::vector<std::pair<int, std::vector<Eigen::Vector3d>>> structurePlaneClouds_;

    cv::Mat currTrackedFeaturesImg_;
    cv::Mat trackHistoryImg_;
    cv::Mat currTrackedDepthImg_;
    cv::Mat currTrackedSegImg_;
    cv::Mat debugImg_;

    std::thread* threadDraw_;
    std::thread*threadDrawC_;

    int imageWidth_ = 640;
    int imageHeight_ = 480;


    std::mutex featMutex_;
    std::mutex imgMutex_;
    std::mutex stateMutex_;

    bool usePlane_ = false;
    bool useDepth_ = false;
    bool autoExit_ = false;
    bool quitFlag_ = false;

    bool showMapID_ = false;
    bool showDebug_ = true;
    bool showCameraView_ = false;

    std::string debugText = "";

    const int colors_bgr[80][3] = {
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

//    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> borders_;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> corners_;
    std::vector<Eigen::Vector3d> directions_;

    const int colors_bgr_axis[3][3] = { {0, 0, 255}, {0, 255, 0}, {255, 0, 0} }; //
};
}



#endif //OV_STEREO_VISUALIZER_H
