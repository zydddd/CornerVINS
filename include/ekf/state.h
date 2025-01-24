/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by Fulin Tang on 2021/1/11.
 */

#ifndef STATE_H
#define STATE_H

#include <map>
#include <vector>
#include <unordered_map>

#include "data_types/basic_type.h"
#include "data_types/imu.h"
#include "data_types/vec.h"
#include "data_types/pose_jpl.h"
#include "data_types/landmark.h"
#include "data_types/corner.h"

#include "plane_feature.h"
#include "plane_graph.h"

namespace Rvg {

/**
 * @brief The class which stores all parameters of the EKF filter
 */
struct StateParameters
{
    /// Bool to determine whether or not to do first estimate Jacobians
    bool do_fej = true;

    /// Bool to determine whether or not use imu message averaging
    bool imu_avg = false;

    /// Bool to determine if we should use Rk4 imu integration
    bool use_rk4_integration = true;

    /// Bool to determine whether or not to calibrate imu-to-camera pose
    bool do_calib_camera_extrinsic = false;

    /// Bool to determine whether or not to calibrate camera intrinsics
    bool do_calib_camera_intrinsics = false;

    /// Bool to determine whether or not to calibrate camera to IMU time offset
    bool do_calib_camera_timeoffset = false;

    bool do_calib_ground = true;

    /// Max clone size of sliding window
    int max_clone_size = 11;

    /// Max number of estimated SLAM features
    int max_slam_features = 25;

    /// Max number of SLAM features we allow to be included in a single EKF update.
    int max_slam_in_update = INT_MAX;

    /// Max number of MSCKF features we will use at a given image timestep.
    int max_msckf_in_update = INT_MAX;

    /// What representation our features are in (msckf features)
    LandmarkRepresentation::Representation feat_rep_msckf = LandmarkRepresentation::Representation::GLOBAL_3D;

    /// What representation our features are in (slam features)
    LandmarkRepresentation::Representation feat_rep_slam = LandmarkRepresentation::Representation::GLOBAL_3D;

    /// marginalize paras
    int track_thres = 5;  //
    int lost_thres = 200;  //
    int obs_thres = 10;  //
    double area_thres = 0.5;  //
    double relocate_area_thres = 1.0;  //

    /// merge paras
    bool do_merge = true;  //
    double angle_thres = 10.0;
    double dis_thres = 0.1;
    double overlap_thres = 0.0;

    bool fix_plane_landmark = false;
    bool fix_corner_landmark = false;
    bool rotation_update_free = true;  //
    bool rotation_update_struct = true;  //
    bool marginalize_corner_plane = false;  //
    bool use_coplanar_corner = false;
};



/**
 * @brief State of the EKF filter
 */
class State
{

public:
    State(StateParameters& parameters_);
    ~State();


    /**
     * @brief Will return the timestep that we will marginalize next.
     */
   double MargTimestep();


   /**
    * @brief Calculates the current max size of the covariance
    */
   int MaxCovarianceSize();


   /// Current timestamp (should be the last update time!)
   double timestamp_;
   double last_timestamp_;

    Eigen::Matrix<double,7,1> gtPose_;


   /// Struct containing filter options
   StateParameters parameters_;

   /// Pointer to the "active" IMU state (q_GtoI, p_IinG, v_IinG, bg, ba)
   std::shared_ptr<IMU> imu_;

   /// Map between imaging times and clone poses (q_GtoIi, p_IiinG)
   std::map<double, std::shared_ptr<PoseJPL> > clonesImu_;  // 窗口

   /// Our current set of SLAM features (3d positions)
   std::unordered_map<size_t, std::shared_ptr<Landmark>> featuresSlam_;

   /// Time offset base IMU to camera (t_imu = t_cam + t_off)
   std::shared_ptr<Vec> calibDtCamToImu_;
   /// The calibration pose between the camera and IMU (R_ItoC, p_IinC)
   std::shared_ptr<PoseJPL> calibImuToCam_;

   /// Camera intrinsics
   std::shared_ptr<Vec> camIntrinsics_;

   /// Which distortion model we are using
   /// false = radtan, true = fisheye
   bool camIntrinsicsModel_ = false;

    /// Our current set of PLANE closest-point features (3d positions in global frame)
    std::unordered_map<int, std::shared_ptr<PlaneCP>> featuresPlane_;
    std::unordered_map<int, std::shared_ptr<Corner>> featuresCorner_;
    std::unordered_map<int, std::shared_ptr<Position>> featuresPosition_;
    std::unordered_map<int, std::shared_ptr<Rotation>> featuresRotation_;
    std::unordered_map<int, std::shared_ptr<StructurePlane>> featuresStructurePlane_;
    std::shared_ptr<FeatureGraph>  graphMap_ = nullptr;

    void SetStatusDebug(std::string str);
    void ClearStatusDebug();
    std::string GetStatusDebug();
    std::string debugInfo = "";



public:
   friend class StateUtils;

   /// Covariance of all active variables
   Eigen::MatrixXd Cov_;

   /// Vector of variables
   std::vector<std::shared_ptr<BasicType> > variables_;
};

}

#endif // STATE_H