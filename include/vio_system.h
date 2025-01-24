/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/19.
 */

#ifndef RGBD_PLANE_VIO_VIO_SYSTEM_H
#define RGBD_PLANE_VIO_VIO_SYSTEM_H

#include <string>
#include <algorithm>
#include <fstream>
#include <memory>
#include <Eigen/StdVector>
#include <opencv2/opencv.hpp>
#include <pangolin/pangolin.h>

#include "point_features/point_klt_tracker.h"
#include "static_initializer.h"
#include "data_types/landmark.h"
#include "ekf/propagator.h"
#include "ekf/state.h"
#include "ekf/state_utils.h"
#include "ekf/update_msckf_feature.h"
#include "ekf/update_slam_feature.h"
#include "ekf/update_plane.h"
#include "ekf/update_zupt.h"
#include "parameters.h"
#include "motion_initializer.h"

#include "plane_features/plane_feature.h"
#include "plane_features/plane_detector.h"
#include "plane_features/plane_graph_tracker.h"
#include "plane_features/feature_associator.h"

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree.h>

#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/impl/normal_3d.hpp>

#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "utils.h"

namespace Rvg {

// Struct for save 3D features
    struct PointCloudId
    {
        size_t id;
        Eigen::Vector3d pos;
        std::vector<double> timestamps;
    };

    class VIOSystem
    {
    public:

        /**
         * @brief Default constructor, will load all configuration variables
         * @param params Parameters loaded from the configure file
         */
        VIOSystem(Parameters &params);

        void ProcessWithGroundTruth(double timestamp, cv::Mat& frame, cv::Mat& depth_image, Eigen::Matrix<double,7,1>& pose);


        /**
         * @brief feed the imu measurement to the system
         */
        void ProcessImuMeasurement(double timestamp, Eigen::Vector3d wm, Eigen::Vector3d am);

        /**
         * @brief feed the camera measurement to the system
         */
        void ProcessCameraMeasurement(double timestamp, cv::Mat& frame);
        void ProcessCameraMeasurement(double timestamp, cv::Mat& frame, cv::Mat& depth);


        /// If we are initialized or not
        bool Initialized();

        /// Timestamp that the system was initialized at
        double InitializedTime();

        /// Accessor for current system parameters
        Parameters GetParams();

        /// Accessor to get the current state
        std::shared_ptr<State> GetState();

        /// Get a nice visualization image of what tracks we have
        cv::Mat GetHistoricalTrackedImage();
        cv::Mat GetCurrentTrackedImage();

        void DetectPlanes(double timestamp);
        /// Get a color image for visualization
        cv::Mat GetHistoricalTrackedImageColor();
        cv::Mat GetCurrentTrackedImageColor();
        cv::Mat GetCurrentTrackedImageDepth();
        cv::Mat GetCurrentTrackedImageSeg();
        /// Returns 3d features used in the last update in global frame
        std::vector<Eigen::Vector3d> GetGoodFeaturesMsckf();
        /// Returns 3d SLAM features in the global frame
        std::vector<Eigen::Vector3d> GetFeaturesSlam();
        /// Return 3d Init features in the global frame
        std::vector<Eigen::Vector3d> GetInitFeatures();
        std::vector<std::tuple<int, int, std::vector<Eigen::Vector3d>>> GetPlaneFeatures();
        std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> GetCorners();
        std::vector<Eigen::Vector3d> GetDirections();
        Eigen::Matrix<double,7,1> GetGtPose();

        std::shared_ptr<Propagator> GetProp();
        /// Return 2d Feature Points
        std::vector<Point_ID> GetPointID();
        /// Return 3D features positions
        std::vector<PointCloudId> Get3DPointsIdSLAM();
        std::vector<PointCloudId> Get3DPointsIdMSCKF();
        /// Return Features Track History
        std::shared_ptr<PointFeatureDatabase> GetTrackHistory();

        /**
         * @brief Given a state, this will initialize our IMU state.
         * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
         */
        void initialize_with_state(Eigen::Matrix<double, 17, 1> imustate);  // for motion initialize



        Eigen::Matrix3f rot_ref2curr_;

        std::string GetCurrentDebugInformation();
    protected:

        /**
         * @brief This will do the propagation and feature updates to the state
         */
        void DoFeaturePropagateUpdate(double timestamp);

        /**
         * @brief This function will try to initialize the state.
         * @return True if we have successfully initialized
         */
        bool InitializeByStill();

        void Denoise(cv::Mat& depth_img);
        void FillGround(cv::Mat& depth);

        void GetFeaturesUpdate(double timestamp,
                           std::vector<std::shared_ptr<PointFeature>> &featsUpMSCKF,
                           std::vector<std::shared_ptr<PointFeature>> &featsSlamDELAYED,
                           std::vector<std::shared_ptr<PointFeature>> &featsSlamUPDATE);


        /// Manager parameters
        Parameters params_;

        /// Our master state object :D
        std::shared_ptr<State> state_;

        /// Propagator of our state
        std::shared_ptr<Propagator> propagator_;

        /// Complete history of our feature tracks
        std::shared_ptr<PointFeatureDatabase> trackDATABASE_;
        /// Our sparse feature tracker
        std::shared_ptr<PointTracker> trackFEATS_;

        std::shared_ptr<PlaneDetector> planeDetector_;

        std::shared_ptr<PlaneTracker> planeTracker_;

        /// State initializer
        std::shared_ptr<StaticInitializer> staticInitializer_;
        /// Motion initializer
        std::shared_ptr<MotionInitializer> motionInitializer_;
        /// Boolean if we are initialized or not
        bool isInitializedVio_ = false;
        /// Boolean we use static or motion initializer
        bool useMotionInit_ = true;

        /// MSCKF feature updater
        std::shared_ptr<UpdateMsckfFeature> updaterMSCKF_;
        /// SLAM feature updater
        std::shared_ptr<UpdateSlamFeature> updaterSLAM_;
        /// Plane updater
        std::shared_ptr<UpdatePlane> updaterPlane_;
        /// Zero Velocity Updater
        bool did_zupt_update = false;
        cv::Mat zuptImage_;
        std::shared_ptr<UpdateZUPT> updaterZUPT_;


        // Time variables
        std::chrono::steady_clock::time_point to, t1, t2, t3, t4, t5, t6, t7;
        double t_imu_integration, t_imu_propagation, t_imu;
        double t_plane_detection, t_corner_detection, t_point_detection, t_point_tracking, t_feature_association;
        double t_point_update, t_plane_update, t_corner_update, t_coplane_update;
        double t_marginalize_plane;
        // Track how much distance we have traveled
        double timeLastUpdate_ = -1;
        double distance_ = 0;

        // Startup time of the filter
        double startupTime_ = -1;


        /// Visualizer
        // Good features that where used in the last update (used in visualization)
        std::vector<Eigen::Vector3d> goodFeaturesMSCKF_;
        std::vector<Eigen::Vector3d> goodFeaturesSLAM_;
        std::vector<Eigen::Vector3d> featuresInit_;
        std::vector<PointCloudId> goodFeaturesIdMSCKF_;

        //  image for visualizer
        cv::Mat colorImage_;
        cv::Mat depthImage_;
        cv::Mat segImage_;


        std::ofstream debug_file, time_file;
        std::thread* threadDetect_;
        std::ofstream plane_file;
        std::shared_ptr<FeatureAssociator> featureAssociator_;

        std::ofstream sp_file;
        std::ofstream ds_file, fit_file;
        std::ofstream merge_file, refit_file;

        std::ofstream map_file, ass_file;

    };

}

#endif // RGBD_PLANE_VIO_VIO_SYSTEM_H
