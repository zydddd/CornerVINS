/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/12.
 */

#ifndef RGBD_PLANE_VIO_UPDATE_MSCKF_FEATURE_H
#define RGBD_PLANE_VIO_UPDATE_MSCKF_FEATURE_H

#include <Eigen/Eigen>

#include "point_features/point_feature.h"
#include "data_types/landmark.h"
#include "point_features/point_triangulation.h"
#include "quat_utils.h"
#include "ekf/state.h"
#include "ekf/state_utils.h"
#include "ekf/update_utility.h"

#include <chrono>
#include <random>

namespace Rvg {


/**
 * @brief Will compute the system for our sparse features and update the filter.
 */
class UpdateMsckfFeature
{

public:

    /**
     * @brief Default constructor for our MSCKF updater
     */
    UpdateMsckfFeature(UpdaterParameters &parameters_, PointTriangulationParameters &feat_init_parameters, std::map<int, double>& chi_square);

    /**
     * @brief Given tracked features, this will try to use them to update the state.
     */
    void Update(std::shared_ptr<State> state, std::vector<std::shared_ptr<PointFeature>>& feature_vec);
    double ComputeMAD(const Eigen::VectorXd &res_big);

protected:

    /// Options used during update
    UpdaterParameters parameters_;

    /// Feature initializer class object
    std::unique_ptr<PointTriangulation> initializerFeat_;

    /// Chi squared 95th percentile table (lookup would be size of residual)
    std::map<int, double> chiSquaredTable_;

    double maxOptIter_ = 3;
    std::ofstream debug_file;
};


}

#endif RGBD_PLANE_VIO_UPDATE_MSCKF_FEATURE_H
