/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/19.
 */

#include "parameters.h"

using namespace RvgVio;

/**
 * @brief This function will load paramters from config file
 * This is the recommended way of loading parameters
 * @param file_path yaml config file path
 * @param file_name yaml config file name
 * @return A fully loaded Parameters object
 */
Parameters Parameters::ReadParametersFromYaml(std::string file_path, std::string file_name, std::string save_path)
{
    // Our vio manager options with defaults
    Parameters params;
    cv::FileStorage file_settings(file_path + file_name, cv::FileStorage::READ);
    params.path_root = save_path;
    file_settings["save_figs"] >> params.save_figs;

    file_settings["upspeed"] >> upspeed;


    // for corners
    file_settings["use_corner"] >> params.use_corner;
    file_settings["use_hm"] >> params.use_hm;
    file_settings["share_rotation"] >> params.share_rotation;

    // Read in depth parameters
    file_settings["has_depth"] >> params.has_depth;
    file_settings["depth_filter"] >> params.depth_filter;
    file_settings["depth_near"] >> params.depth_near;
    file_settings["depth_far"] >> params.depth_far;
    file_settings["depth_thres"] >> params.depth_thres;
    file_settings["depth_factor"] >> params.depth_factor;

    // Read in plane parameters
    file_settings["use_plane"] >> params.use_plane;

    file_settings["plane_noise"] >> params.plane_noise;
    file_settings["up_plane_chi2_multipler"] >> params.plane_chi2_multipler;
    file_settings["up_min_measurements"] >> params.up_min_measurements;

    file_settings["use_tracking_matcher"] >> params.use_tracking_matcher;
    file_settings["use_drift_suppression"] >> params.use_drift_suppression;


    /// Thresholds for associating planes
    file_settings["det_angle_thres"] >> params.det_angle_thres;
    file_settings["det_dis_thres"] >> params.det_dis_thres;


    file_settings["angle_thres"] >> params.angle_thres;
    file_settings["dis_thres"] >> params.dis_thres;
    file_settings["overlap_thres"] >> params.overlap_thres;

    file_settings["merge_angle_thres"] >> params.merge_angle_thres;
    file_settings["merge_dis_thres"] >> params.merge_dis_thres;
    file_settings["merge_overlap_thres"] >> params.merge_overlap_thres;


    file_settings["binary_angle_thres"] >> params.binary_angle_thres;
    file_settings["binary_dis_thres"] >> params.binary_dis_thres;
    file_settings["binary_overlap_thres"] >> params.binary_overlap_thres;
    file_settings["drift_angle_thres"] >> params.drift_angle_thres;
    file_settings["drift_dis_thres"] >> params.drift_dis_thres;
    file_settings["drift_overlap_thres"] >> params.drift_overlap_thres;
    file_settings["drift_plane_num"] >> params.drift_plane_num;

    // Merge
    file_settings["do_merge"] >> params.state_params.do_merge;
    file_settings["angle_thres"] >> params.state_params.angle_thres;
    file_settings["dis_thres"] >> params.state_params.dis_thres;
    file_settings["overlap_thres"] >> params.state_params.overlap_thres;

    // Marginalize
    file_settings["track_thres"] >> params.state_params.track_thres;
    file_settings["lost_thres"] >> params.state_params.lost_thres;
    file_settings["obs_thres"] >> params.state_params.obs_thres;
    file_settings["area_thres"] >> params.state_params.area_thres;

    // chisquare check ======================================================================
    std::map<int, double> chi_square_table_tmp;
    std::string chi_square_file = file_path + file_settings["chisquare_file"];
    std::ifstream fin_chisquare;
    fin_chisquare.open(chi_square_file);
    if(!fin_chisquare.is_open()) {
        std::cerr << "------------Reading Chisquare Check file error--------------" << std::endl;
    }
    while(!fin_chisquare.eof()) {
        std::string str_tmp;
        getline(fin_chisquare, str_tmp);

        if(!str_tmp.empty()) {
            std::stringstream ss;
            ss << str_tmp;

            int index;
            double num;
            ss >> index;
            ss >> num;

            chi_square_table_tmp[index] = num;
        }
    }
    fin_chisquare.close();
    params.chi_square_table = chi_square_table_tmp;

    // ESTIMATOR ======================================================================
    // Main EKF parameters
    file_settings["use_fej"] >> params.state_params.do_fej;
    file_settings["use_imuavg"] >> params.state_params.imu_avg;
    file_settings["use_rk4int"] >> params.state_params.use_rk4_integration;
    file_settings["calib_cam_extrinsics"] >> params.state_params.do_calib_camera_extrinsic;
    file_settings["calib_cam_intrinsics"] >> params.state_params.do_calib_camera_intrinsics;
    file_settings["calib_cam_timeoffset"] >> params.state_params.do_calib_camera_timeoffset;
    file_settings["max_clones"] >> params.state_params.max_clone_size;
    file_settings["max_slam"] >> params.state_params.max_slam_features;
    file_settings["max_slam_in_update"] >> params.state_params.max_slam_in_update;
    file_settings["max_msckf_in_update"] >> params.state_params.max_msckf_in_update;
    file_settings["dt_slam_delay"] >> params.dt_slam_delay;


    // Read in what representation our feature is
    std::string feat_rep_msckf_str = "GLOBAL_3D";
    std::string feat_rep_slam_str = "GLOBAL_3D";
    file_settings["feat_rep_msckf"] >> feat_rep_msckf_str;
    file_settings["feat_rep_slam"] >> feat_rep_slam_str;
    // Set what representation we should be using
    std::transform(feat_rep_msckf_str.begin(), feat_rep_msckf_str.end(),feat_rep_msckf_str.begin(), ::toupper);
    std::transform(feat_rep_slam_str.begin(), feat_rep_slam_str.end(),feat_rep_slam_str.begin(), ::toupper);
    params.state_params.feat_rep_msckf = LandmarkRepresentation::FromString(feat_rep_msckf_str);
    params.state_params.feat_rep_slam = LandmarkRepresentation::FromString(feat_rep_slam_str);
    if (params.state_params.feat_rep_msckf == LandmarkRepresentation::Representation::UNKNOWN ||
        params.state_params.feat_rep_slam == LandmarkRepresentation::Representation::UNKNOWN)
    {
        std::cerr << "invalid feature representation specified!" << std::endl;
        std::cerr << "The valid types are:" << std::endl;
        std::cerr << "------ GLOBAL_3D ------" << std::endl;
        std::cerr << "------ ANCHORED_FULL_INVERSE_DEPTH ------" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    // Filter static initialization
    file_settings["init_window_time"] >> params.init_window_time;
    file_settings["init_imu_thresh"] >> params.init_imu_thresh;

    /// use motion initalizer or not
    file_settings["use_motion_init"] >> params.use_motion_init;
    file_settings["use_init_optimization"] >> params.use_init_optimization;
    file_settings["optimize_init_scale"] >> params.optimize_init_scale;
    file_settings["min_parallax"] >> params.min_parallax;
    file_settings["relative_corres"] >> params.relative_corres;
    file_settings["relative_parallax"] >> params.relative_parallax;


    // NOISE ======================================================================
    // Our noise values for inertial sensor
    file_settings["gyroscope_noise_density"] >> params.imu_noises.sigma_w;
    file_settings["accelerometer_noise_density"] >> params.imu_noises.sigma_a;
    file_settings["gyroscope_random_walk"] >> params.imu_noises.sigma_wb;
    file_settings["accelerometer_random_walk"] >> params.imu_noises.sigma_ab;

    // Read in update parameters
    file_settings["up_msckf_sigma_px"] >> params.msckf_params.sigma_pix;
    file_settings["up_msckf_chi2_multipler"] >> params.msckf_params.chi2_multipler;
    file_settings["up_msckf_use_mad"] >> params.msckf_params.use_mad;

    if(params.has_depth)
    {
        file_settings["up_msckf_with_depth"] >> params.msckf_params.with_depth;
    }
    else
    {
        params.msckf_params.with_depth = false;
    }

    file_settings["up_slam_sigma_px"] >> params.slam_params.sigma_pix;
    file_settings["up_slam_chi2_multipler"] >> params.slam_params.chi2_multipler;
    file_settings["up_slam_use_mad"] >> params.slam_params.use_mad;

    if(params.has_depth)
    {
        file_settings["up_slam_with_depth"] >> params.slam_params.with_depth;
    }
    else
    {
        params.slam_params.with_depth = false;
    }

    // Read in ZUPT parameters
    file_settings["use_zupt"] >> params.use_zupt;
    file_settings["zupt_max_velocity"] >> params.zupt_max_velocity;
    file_settings["zupt_noise_multiplier"] >> params.zupt_noise_multiplier;
    file_settings["zupt_chi2_multipler"] >> params.zupt_params.chi2_multipler;
    file_settings["zupt_sigma_px"] >> params.zupt_params.sigma_pix;


    // STATE ======================================================================
    // Timeoffset from camera to IMU
    file_settings["calib_camimu_dt"] >> params.calib_camimu_dt;
    // Global gravity
    std::vector<double> gravity = {params.gravity(0), params.gravity(1), params.gravity(2)};
    cv::Mat gravity_mat;
    file_settings["gravity"] >> gravity_mat;
    assert(gravity.size()==3);
    params.gravity << gravity_mat.at<double>(0,0), gravity_mat.at<double>(1,0), gravity_mat.at<double>(2,0);


    // TRACKERS ======================================================================
    // Tracking flags
    file_settings["multi_threading"] >> params.use_multi_threading;
    // General parameters
    file_settings["num_pts"] >> params.num_pts;
    file_settings["fast_threshold"] >> params.fast_threshold;
    file_settings["grid_x"] >> params.grid_x;
    file_settings["grid_y"] >> params.grid_y;
    file_settings["min_px_dist"] >> params.min_px_dist;
//    file_settings["knn_ratio"] >> params.knn_ratio;

    file_settings["tri_min_dist"] >> params.featinit_params.min_dist;
    file_settings["tri_max_dist"] >> params.featinit_params.max_dist;
    if(params.has_depth)
    {
        file_settings["tri_use_depth"] >> params.featinit_params.use_depth;
    }
    else
    {
        params.featinit_params.use_depth = false;
    }

    // ====================================================================================
    std::vector<int> matrix_wh;
    matrix_wh.push_back(file_settings["cam_width"]);
    matrix_wh.push_back(file_settings["cam_height"]);
    assert(matrix_wh.size() == 2);
    std::pair<int,int> wh(matrix_wh.at(0), matrix_wh.at(1));

    // Camera intrinsic properties
    Eigen::Matrix<double,8,1> cam_calib;
    std::vector<double> matrix_k, matrix_d;
    cv::Mat matrix_k_mat, matrix_d_mat;
    file_settings["cam_k"] >> matrix_k_mat;
    file_settings["cam_d"] >> matrix_d_mat;
    for (int i = 0; i < 4; i++)
    {
        matrix_k.push_back(matrix_k_mat.at<double>(i,0));
        matrix_d.push_back(matrix_d_mat.at<double>(i,0));
    }

    cam_calib << matrix_k.at(0),matrix_k.at(1),matrix_k.at(2),matrix_k.at(3),matrix_d.at(0),matrix_d.at(1),matrix_d.at(2),matrix_d.at(3);

    // Our camera extrinsics transform
    cv::Mat matrix_TCtoI_mat;
    file_settings["T_CtoI"] >> matrix_TCtoI_mat;
    params.T_CtoI << matrix_TCtoI_mat.at<double>(0,0),matrix_TCtoI_mat.at<double>(0,1),matrix_TCtoI_mat.at<double>(0,2),matrix_TCtoI_mat.at<double>(0,3),
                     matrix_TCtoI_mat.at<double>(1,0),matrix_TCtoI_mat.at<double>(1,1),matrix_TCtoI_mat.at<double>(1,2),matrix_TCtoI_mat.at<double>(1,3),
                     matrix_TCtoI_mat.at<double>(2,0),matrix_TCtoI_mat.at<double>(2,1),matrix_TCtoI_mat.at<double>(2,2),matrix_TCtoI_mat.at<double>(2,3),
                     matrix_TCtoI_mat.at<double>(3,0),matrix_TCtoI_mat.at<double>(3,1),matrix_TCtoI_mat.at<double>(3,2),matrix_TCtoI_mat.at<double>(3,3);

    // Load these into our state
    ///
    Eigen::Matrix<double,7,1> cam_eigen;
    cam_eigen.block(0,0,4,1) = QuatUtils::Rot2Quat(params.T_CtoI.block(0,0,3,3).transpose());
    cam_eigen.block(4,0,3,1) = -params.T_CtoI.block(0,0,3,3).transpose() * params.T_CtoI.block(0,3,3,1);

    params.camera_wh = wh;
    params.camera_intrinsics = cam_calib;
    params.camera_extrinsics = cam_eigen;

    // Success, lets returned the parsed options
    return params;
}
