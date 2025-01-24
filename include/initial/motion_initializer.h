//
// Created by weihao on 2021/8/9.
//

#ifndef RVG_VIO_VIS_MOTION_INITIALIZER_H
#define RVG_VIO_VIS_MOTION_INITIALIZER_H

#include "point_features/point_klt_tracker.h"
#include "ekf/propagator.h"

#include "feature_manager.h"
#include "global_sfm.h"
#include "integration_base.h"
#include "initial_utility.h"
#include "motion_estimator.h"
#include "visual_imu_alignment.h"

#include "parameters.h"

#include <ceres/ceres.h>
#include "pose_local_parameterization.h"
#include "imu_factor.h"
#include "projection_factor.h"

namespace Rvg {

class MotionInitializer
{

public:
    /**
     * @brief Default Constructor
     * */
    MotionInitializer(Parameters &params);

    /**
     * @brief Reset all parameters
     * */
     void Reset();

    /**
     * @brief Input IMU measurement
     * @param timestamp
     * @param acc
     * @param gyro
     * */
    void processIMU(const ImuData &data);

    /**
     * @brief Input Image measurement
     * @param timestamp image's timestamp
     * @param src the input source image
     * @param track_feats feature tracker class (to get corresponding points)
     * */
     void processImage(double timestamp, cv::Mat& src, std::shared_ptr<PointTracker>& track_feats);
     void processImage(double timestamp, cv::Mat& src, std::shared_ptr<PointTracker>& track_feats,
                       double& expo_time, double& rst_time, double& offset_time);



     /// check initial state
     bool isInitialized() {
         return initial_state;
     }

    bool initSuccess(std::vector<Eigen::Matrix<double, 17, 1>>& init_state)
    {
        if(system_init_label)
            init_state = system_init_state;
        return system_init_label;
    }

    bool getFeaturesPosition(std::map<int, Eigen::Vector3d>& feats_in_world){
        if(system_init_label)
            feats_in_world = current_feats_in_world;
        return system_init_label;
     }

    bool initialStructure();
    bool visualInitialAlign();
    bool relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l);

    // build slide window
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();

    // graph optimization
    void solveOdometry();
    void optimization();
    void vector2double();
    void double2vector();

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    MarginalizationFlag  marginalization_flag;
    RvgVig::FeatureManager f_manager;

    VisualImuAlignment align;

protected:

    // internal parameters
    std::map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;
    int frame_count;
    double initial_timestamp;
    int WINDOW_SIZE = 10;
    bool initial_state = false;

    // optimization option
    int NUM_ITERATIONS = 100;
    double SOLVER_TIME = 0.04;

    // slide window pose
    Eigen::Vector3d Ps[(10 + 1)];
    Eigen::Vector3d Vs[(10 + 1)];
    Eigen::Matrix3d Rs[(10 + 1)];
    Eigen::Vector3d Bas[(10 + 1)];
    Eigen::Vector3d Bgs[(10 + 1)];
    double td;
    double scale;

    // Initializer Pose
    Eigen::Matrix3d back_R0, last_R, last_R0;
    Eigen::Vector3d back_P0, last_P, last_P0;

    double Headers[(10 + 1)];

    /// Our history of IMU messages (time, angular, linear)
    std::vector<ImuData> imuDataset;

    Eigen::Matrix2d project_sqrt_info_;
    Eigen::Vector3d g;
    Eigen::Vector3d inputG;

    std::vector<Eigen::Matrix3d> RIC;
    std::vector<Eigen::Vector3d> TIC;

    Eigen::Matrix3d ric[1];
    Eigen::Vector3d tic[1];

    IntegrationBase *pre_integrations[(10 + 1)];
    Eigen::Vector3d acc_0, gyr_0;

    std::vector<double> dt_buf[(10 + 1)];
    std::vector<Eigen::Vector3d> linear_acceleration_buf[(10 + 1)];
    std::vector<Eigen::Vector3d> angular_velocity_buf[(10 + 1)];

    int  sum_of_back, sum_of_front;

    MotionEstimator m_estimator;

    double acc_n, gyro_n, acc_w, gyro_w;

    // last image timestamp
    double last_image_time = -1;

    // parameter for optimization
    double para_Pose[10 + 1][7];
    double para_SpeedBias[10 + 1][9];
    double para_Feature[1000][1];
    double para_Ex_Pose[1][7];
    double para_Retrive_Pose[7];
    double para_Td[1][1];
    double para_Tr[1][1];
    double para_S[1][1];

    // motion init state
    bool system_init_label = false;
    std::vector<Eigen::Matrix<double, 17, 1>> system_init_state;
    std::map<int, Eigen::Vector3d> features_in_world;
    std::map<int, Eigen::Vector3d> current_feats_in_world;

    Eigen::Matrix3d rot_CO2G;

    bool use_init_optimization = false;
    bool optimize_init_scale = false;
    double fx;

    double last_minus_ = 0.0;

};
}
#endif //RVG_VIO_VIS_MOTION_INITIALIZER_H
