/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/19.
 */

#ifndef RGBD_PLANE_VIO_PARAMETERS_H
#define RGBD_PLANE_VIO_PARAMETERS_H

#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "ekf/state.h"
#include "ekf/propagator.h"
#include "ekf/update_utility.h"
#include "point_features/point_triangulation.h"
#include "quat_utils.h"

namespace Rvg {

/**
 * @brief Struct which stores all options needed for state estimation.
  */
    struct Parameters
    {

        bool save_figs = false;
        bool use_corner = false;

        std::string path_root = "./";
        bool use_tracking_matcher = false;
        bool use_drift_suppression = false;

        /// Options for depth
        bool has_depth = false;

        /// Thresholds for detecting planes
        double det_angle_thres = 5.0;
        double det_dis_thres = 0.02;
        double plane_noise = 1;
        bool use_hm = false;
        bool share_rotation = false;

        /// Thresholds for associating planes
        double angle_thres = 10.0;
        double dis_thres = 0.1;
        double overlap_thres = 0.0;

        /// Thresholds for merging planes
        double merge_angle_thres = 5.0;
        double merge_dis_thres = 0.05;
        double merge_overlap_thres = 0.0;

        /// Thresholds for binary check
        double binary_angle_thres = 10.0;
        double binary_dis_thres = 0.5;
        double binary_overlap_thres = 0.5;

        /// Thresholds for drift detection
        double drift_angle_thres = 15.0;
        double drift_dis_thres = 0.5;
        double drift_overlap_thres = 0.2;
        int drift_plane_num = 2;
        ushort depth_near = 500;
        ushort depth_far = 2500;
        ushort depth_thres = 100;
        double depth_factor = 1000.0;

        bool depth_filter = false;
        /// Update options for plane
        bool use_plane = false;

        int plane_chi2_multipler = 5;
        int up_min_measurements = 3;


        // ESTIMATOR ===============================
        /// Core state options (e.g. number of cameras, use fej, stereo, what calibration to enable etc)
        StateParameters state_params;

        /// Delay, in seconds, that we should wait from init before we start estimating SLAM features
        double dt_slam_delay = 2.0;

        /// Amount of time we will initialize over (seconds)
        double init_window_time = 1.0;
        ///  Variance threshold on our acceleration to be classified as moving
        double init_imu_thresh = 1.0;

        /// use motion initializer or not
        bool use_motion_init = false;
        bool use_init_optimization = false;
        bool optimize_init_scale = false;
        double min_parallax = 10.0;
        int relative_corres = 20;
        double relative_parallax = 30.0;


        // NOISE / CHI2 ============================
        /// IMU noise (gyroscope and accelerometer)
        Propagator::NoiseParameters imu_noises;
        /// Update options for MSCKF features (pixel noise and chi2 multiplier)
        UpdaterParameters msckf_params;
        /// Update options for SLAM features (pixel noise and chi2 multiplier)
        UpdaterParameters slam_params;

        /// Update options for ZUPT
        bool use_zupt = false;
        double zupt_max_velocity = 1.0;
        double zupt_noise_multiplier = 1.0;
        UpdaterParameters zupt_params;


        /// Chi squared 95th percentile table (lookup would be size of residual)
        std::map<int, double> chi_square_table;  //


        // STATE DEFAULTS ==========================
        /// Gravity in the global frame (i.e. should be [0, 0, 9.81] typically)
        Eigen::Vector3d gravity = {0.0, 0.0, 9.81};
        /// Time offset between camera and IMU.
        double calib_camimu_dt = 0.0;
        Eigen::Matrix4d T_CtoI;
        Eigen::VectorXd camera_intrinsics;
        Eigen::VectorXd camera_extrinsics;  // camera extrinsics (q_ItoC, p_IinC).
        /// the dimensions of incoming images (width/cols, height/rows). This is normally only used during simulation.
        std::pair<int,int> camera_wh;



        // TRACKERS ===============================
        /// Will use multi-threads to detect features
        bool use_multi_threading = false;
        /// The number of points we should extract and track in *each* image frame. This highly effects the computation required for tracking.
        int num_pts = 150;
        /// Fast extraction threshold
        int fast_threshold = 20;
        /// Number of grids we should split column-wise to do feature extraction in
        int grid_x = 5;
        /// Number of grids we should split row-wise to do feature extraction in
        int grid_y = 5;
        /// Will check after doing KLT track and remove any features closer than this
        int min_px_dist = 10;
//        double knn_ratio = 0.75;

        /// Parameters used by our feature initialize / triangulator
        PointTriangulationParameters featinit_params;


        static Parameters ReadParametersFromYaml(std::string file_path, std::string file_name, std::string save_path);

    };
}

#endif //RGBD_PLANE_VIO_PARAMETERS_H
