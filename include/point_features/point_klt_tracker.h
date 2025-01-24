/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/11.
 */

#ifndef RGBD_PLANE_VIO_POINT_KLT_TRACKER_H
#define RGBD_PLANE_VIO_POINT_KLT_TRACKER_H

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "point_detector.h"
#include "point_feature_database.h"
#include "plane_feature.h"

namespace Rvg {

struct Point_ID {
    cv::Point2f pt;
    size_t id;
};


class PointTracker
{
public:

    /**
     * @brief Public default constructor
     */
    PointTracker() : database_(new PointFeatureDatabase()), numFeatures_(200), useMultiThreading_(false), currid_(0),
                     threshold_(10), gridX_(8), gridY_(5), minPxDist_(30) {}

    /**
     * @brief Public constructor with configuration variables
     * @param numfeats number of features we want want to track (i.e. track 200 points from frame to frame)
     * @param multithread if we should try to process with multiple threads or single threaded
     */
    explicit PointTracker(int numfeats, bool multithread, int fast_threshold, int gridx, int gridy, int minpxdist) :
            database_(new PointFeatureDatabase()), numFeatures_(numfeats), useMultiThreading_(multithread),
            threshold_(fast_threshold), gridX_(gridx), gridY_(gridy), minPxDist_(minpxdist)
    {
        currid_ = (size_t) 0;
//        debug_file.open("/home/zyd/Code/Corner-VIO/bin/depth.txt", std::ios::out | std::ios::trunc);
//        debug_file << std::fixed << std::setprecision(17);
    }


    virtual ~PointTracker() { }

    /**
     * @brief Process a new monocular image
     * @param timestamp timestamp the new image occurred at
     * @param img new cv:Mat grayscale image
     */
    void ProcessMonocular(double timestamp, cv::Mat &img);
    void ProcessMonocular(double timestamp, cv::Mat &img, cv::Mat & depth);
    void ProcessMonocular(double timestamp, cv::Mat &img, cv::Mat & depth, cv::Mat &seg_img);

    void SetDepthParams(ushort depth_near, ushort depth_far, double depth_factor)
    {
        depth_near_ = depth_near;
        depth_far_ = depth_far;
        depth_factor_ = depth_factor;
    }

    /**
     * @brief Shows features extracted in the last image
     * @param img_out image to which we will overlayed features on
     * @param r1,g1,b1 first color to draw in
     * @param r2,g2,b2 second color to draw in
     */
    void DisplayActive(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2);
    // Display Color Image
    void DisplayActiveColor(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2);
    void DrawDelaunay(cv::Mat& img, std::unordered_map<size_t, std::vector<cv::Point2f>>& plane_pts);

    /**
     * @brief Shows a "trail" for each feature (i.e. its history)
     * @param img_out image to which we will overlayed features on
     * @param r1,g1,b1 first color to draw in
     * @param r2,g2,b2 second color to draw in
     * @param highlighted unique ids which we wish to highlight
     */
    void DisplayHistory(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2,
            std::vector<size_t> highlighted={});
    /// Display Color Image
    void DisplayHistoryColor(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2);



    /**
     * @brief Given a the camera intrinsic values, this will set what we should normalize points with.
     * This will also update the feature database with corrected normalized values.
     * Normally this would only be needed if we are optimizing our camera parameters, and thus should re-normalize.
     * @param camera_calib Calibration parameters for all cameras [fx,fy,cx,cy,d1,d2,d3,d4]
     * @param correct_active If we should re-undistort active features in our database
     */
    void SetCalibration(Eigen::VectorXd camera_calib, bool correct_active=false);

    /**
     * @brief Get the feature database with all the track information
     * @return FeatureDatabase pointer that one can query for features
     */
    std::shared_ptr<PointFeatureDatabase> GetFeatureDatabase()
    {
        return database_;
    }

    /**
     * @brief Changes the ID of an actively tracked feature to another one
     * @param id_old Old id we want to change
     * @param id_new Id we want to change the old id to
     */
    void ChangeFeatId(size_t id_old, size_t id_new);

    /**
     * @brief Main function that will undistort/normalize a point.
     * @param pt_in uv 2x1 point that we will undistort
     * @return undistorted 2x1 point
     */
    cv::Point2f UndistortPoint(cv::Point2f pt_in);

    /**
     * @brief Return 2D feature points with id
     * */
    void GetLastPointId(std::vector<Point_ID>& pointsId)
    {
         pointsId.clear();
         assert(idsLast_.size() == ptsLast_.size());
         for(int i = 0; i < idsLast_.size(); i++) {
             Point_ID tmp;
             tmp.pt = ptsLast_[i].pt;
             tmp.id = idsLast_[i];
             pointsId.push_back(tmp);
         }
    }

    // Timing variables
    std::chrono::steady_clock::time_point rT1, rT2, rT3, rT4, rT5, rT6, rT7;
protected:
    /**
     * @brief Detects new features in the current image
     * @param img0pyr image we will detect features on (first level of pyramid)
     * @param pts0 vector of currently extracted keypoints in this image
     * @param ids0 vector of feature ids for each currently extracted keypoint
     *
     * Given an image and its currently extracted features, this will try to add new features if needed.
     * Will try to always have the "max_features" being tracked through KLT at each timestep.
     * Passed images should already be grayscaled.
     */
    void PerformDetectionMonocular(const std::vector<cv::Mat> &img0pyr, std::vector<cv::KeyPoint> &pts0,
            std::vector<size_t> &ids0);
    bool InRect(cv::Point2f pt, cv::Rect rect);
    /**
     * @brief KLT track between two images, and do RANSAC afterwards
     * @param img0pyr starting image pyramid
     * @param img1pyr image pyramid we want to track too
     * @param pts0 starting points
     * @param pts1 points we have tracked
     * @param id0 id of the first camera
     * @param id1 id of the second camera
     * @param mask_out what points had valid tracks
     *
     * This will track features from the first image into the second image.
     * The two point vectors will be of equal size, but the mask_out variable will specify which points are good or bad.
     * If the second vector is non-empty, it will be used as an initial2 guess of where the keypoints are in the second image.
     */
    void PerformMatching(const std::vector<cv::Mat> &img0pyr, const std::vector<cv::Mat> &img1pyr,
            std::vector<cv::KeyPoint> &pts0, std::vector<cv::KeyPoint> &pts1, std::vector<uchar> &mask_out);

    double GetDepth(cv::Mat &depth, cv::Point2f pt);
    size_t GetPlaneId(cv::Mat &seg, cv::Point2f pt);

    /// Database with all our current features
    std::shared_ptr<PointFeatureDatabase> database_;

    /// Set Camera Parameters or Not
    bool cameraParamsFlag_ = false;

    /// Camera intrinsics in OpenCV format
    cv::Matx33d cameraKOpenCV_;

    /// Camera distortion in OpenCV format
    cv::Vec4d cameraDistOpenCV_;

    /// Number of features we should try to track frame to frame
    int numFeatures_;

    /// Boolean for if we should try to multi-thread our frontends
    bool useMultiThreading_ = false;

    /// Mutexs for our last set of image storage (img_last, pts_last, and ids_last)
    std::mutex mtxFeeds_;

    /// Last set of images (use map so all trackers render in the same order)
    cv::Mat imgLast_;
    cv::Mat segLast_;
    /// Last set of tracked points
    std::vector<cv::KeyPoint> ptsLast_;

    /// Set of IDs of each current feature in the database
    std::vector<size_t> idsLast_;
    std::vector<size_t> plidsLast_;

    std::unordered_map<size_t, std::vector<cv::Point2f>> plane_pts_;

    /// Master ID for this tracker (atomic to allow for multi-threading)
    std::atomic<size_t> currid_;



    // Parameters for our FAST grid detector
    int threshold_;
    int gridX_;
    int gridY_;

    // Minimum pixel distance to be "far away enough" to be a different extracted feature
    int minPxDist_;

    // How many pyramid levels to track on and the window size to reduce by
    int pyrLevels_ = 3;
    cv::Size winSize_ = cv::Size(15, 15);

    // Last set of image pyramids
    std::vector<cv::Mat> imgPyramidLast_;

    ushort depth_near_ = 500;
    ushort depth_far_ = 2500;
    ushort depth_thres_ = 100;
    double depth_factor_ = 1000.0;

    cv::Scalar colors_[10] =
            {
                    {255, 0, 0},
                    {255, 255, 0},
                    {100, 20, 50},
                    {0, 30, 255},
                    {10, 255, 60},
                    {80, 10, 100},
                    {0, 255, 200},
                    {10, 60, 60},
                    {255, 0, 128},
                    {60, 128, 128}
            };

    std::ofstream debug_file;
};


}



#endif //RGBD_PLANE_VIO_POINT_KLT_TRACKER_H
