//
// Created by zyd on 23-1-2.
//

#include "update_plane.h"
#include <iomanip>

using namespace RvgVio;

UpdatePlane::UpdatePlane(double noise, std::map<int, double>& chi_square,
            int chi2_multipler, int up_meas): planeNoise_(noise), chiSquaredTable_(chi_square),
                                              chi2_multipler_(chi2_multipler), min_measurements_(up_meas)
                                              {
//                                                  debug_file.open("/home/zyd/Code/Corner-VIO/bin/update.txt", std::ios::out | std::ios::trunc);
//                                                  debug_file << std::fixed << std::setprecision(18);
                                              }

void UpdatePlane::Update(std::shared_ptr<State> state, std::vector<std::shared_ptr<PlaneObservation>>& feature_vec)
{

//    if ((int)state->clonesImu_.size() < std::min(state->parameters_.max_clone_size , 5))
//    {
//        std::cout << "[Update plane] waiting for enough clone state_s (" << (int)state->clonesImu_.size() << " of " << std::min(state->parameters_.max_clone_size, 5) << " ) ......" << std::endl;
//        return;
//    }

    // Return if no features
    if(feature_vec.empty()) {
        return;
    }

    std::cout << "======== Update Planes ========" << std::endl;
    std::cout << "[Update Plane] Feature size: " << feature_vec.size() << std::endl;

    // 0. Get all timestamps our clones are at (and thus valid measurement times)
    std::vector<double> clonetimes;
    for (const auto& clone_imu : state->clonesImu_) {
        clonetimes.emplace_back(clone_imu.first);
    }

    // Calculate the max possible measurement size
    size_t max_meas_size =  3 * feature_vec.size() * (clonetimes.size() + 1);
    // Calculate max possible state size (i.e. the size of our covariance)
    size_t max_hx_size = state->MaxCovarianceSize();

    // Large Jacobian and residual of *all* features for this update
    Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
    Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
    Eigen::MatrixXd R_big = Eigen::MatrixXd::Zero(max_meas_size, max_meas_size);

    std::unordered_map<std::shared_ptr<BasicType>, size_t> Hx_mapping;
    std::vector<std::shared_ptr<BasicType>> Hx_order_big;
    size_t ct_jacob = 0;
    size_t ct_meas = 0;

    std::vector<int> toUpdateMapPlane;


    for(int i = 0; i < feature_vec.size(); i++)
    {
        /// Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
        Eigen::MatrixXd H_f;
        Eigen::MatrixXd H_x;
        Eigen::VectorXd res;
        Eigen::MatrixXd R;
        std::vector<std::shared_ptr<BasicType> > Hx_order;

        int map_id = feature_vec[i]->map_id_;
        std::shared_ptr<PlaneFeature> plane = state->graphMap_->GetPlane(map_id);

        if(this_use_corner_update_ && !reuse_corner_plane && plane->is_corner_) continue;

        /// Get valid measurements
        std::vector<int> timestamp_index;
        plane->GetValidMeasurements(clonetimes, timestamp_index);
        if(timestamp_index.size() < min_measurements_)
            continue;


        std::shared_ptr<BasicType> feat;
        if(plane->dir_id_ != -1)
        {
            std::shared_ptr<StructurePlane> planeFeat = state->featuresStructurePlane_.at(map_id);
            std::cout << "[Update Plane] update for structure plane [" << map_id << "]" << std::endl;

            if(this_use_corner_update_ && !state->parameters_.rotation_update_struct)
            {
                GetStructurePlaneJacobianP(state, plane, timestamp_index, H_f, H_x, res, Hx_order, R);  // Get the Jacobian for this feature
            }
            else
            {
                GetStructurePlaneJacobianFull(state, plane, timestamp_index, H_f, H_x, res, Hx_order, R);  // Get the Jacobian for this feature
            }

            feat = planeFeat;
        }
        else // free plane 3 dof
        {
            std::shared_ptr<PlaneCP> planeFeat = state->featuresPlane_.at(map_id);
            std::cout << "[Update Plane] update for free plane [" << map_id << "]" << std::endl;
            if(this_use_corner_update_ && !state->parameters_.rotation_update_free)
            {
                GetFreePlaneJacobianP(state, plane, timestamp_index, H_f, H_x, res, Hx_order, R);  // Get the Jacobian for this feature
            }
            else
            {
                GetFreePlaneJacobianFull(state, plane, timestamp_index, H_f, H_x, res, Hx_order, R);  // Get the Jacobian for this feature
            }

            feat = planeFeat;
        }

        /// Place Jacobians in one big Jacobian, since the landmark is already in our state vector
        Eigen::MatrixXd H_xf = H_x;
        std::vector<std::shared_ptr<BasicType>> Hxf_order = Hx_order;

        if(plane->is_fixed_)
        {
            std::cout << "(fix plane landmark!)" << std::endl;
        }
        else
        {
            // Else we have the full feature in our state, so just append it
            H_xf.conservativeResize(H_x.rows(), H_x.cols() + H_f.cols());
            H_xf.block(0, H_x.cols(), H_x.rows(), H_f.cols()) = H_f;
            // Append to our Jacobian order vector
            Hxf_order.push_back(feat);
            std::cout << "(refine plane landmark!)" << std::endl;
        }

        /// Chi2 distance check
        Eigen::MatrixXd P_marg = StateUtils::GetMarginalCovariance(state, Hxf_order);
        Eigen::MatrixXd S = H_xf * P_marg * H_xf.transpose();
        S += R;
//        S.diagonal() += planeNoise_ * Eigen::VectorXd::Ones(S.rows());
        double chi2 = res.dot(S.llt().solve(res));

        // Get our threshold (we precompute up to 500 but handle the case that it is more)
        double chi2_check;  //
        if (res.rows() < chiSquaredTable_.size()) {
            chi2_check = chiSquaredTable_[res.rows()];
        } else {
            continue;
        }

//        double noise = R.trace();
        // Check if we should delete or not
        if(chi2 > chi2_multipler_ * chi2_check)
        {
            // 非正常更新
            std::cout << "[Update] (" << plane->map_id_ << ") chi2 check failed." << chi2 << "/ " << chi2_check << std::endl;
            continue;
        }

        /// We are good!!! Append to our large H vector
        size_t ct_hx = 0;
        for (const auto &var : Hxf_order)
        {
            // Ensure that this variable is in our Jacobian
            if (Hx_mapping.find(var)==Hx_mapping.end()) {
                Hx_mapping.insert({var,ct_jacob});
                Hx_order_big.push_back(var);
                ct_jacob += var->GetSize();
            }
            // Append to our large Jacobian
            Hx_big.block(ct_meas, Hx_mapping[var], H_xf.rows(), var->GetSize()).noalias() = H_xf.block(0, ct_hx, H_xf.rows(),var->GetSize());
            ct_hx += var->GetSize();
        }

        // Append our residual and move forward
        res_big.block(ct_meas,0,res.rows(),1).noalias() = res;
        R_big.block(ct_meas, ct_meas, res.rows(), res.rows()).noalias() = R;
        ct_meas += res.rows();
        toUpdateMapPlane.push_back(map_id);
    }  // feature_vec

    // We have appended all features to our Hx_big, res_big
    // Delete it so we do not reuse information

    // Return if we don't have anything and resize our matrices
    if (ct_meas < 1) {
        return;
    }

    assert(ct_meas <= max_meas_size);
    assert(ct_jacob <= max_hx_size);

    res_big.conservativeResize(ct_meas, 1);
    Hx_big.conservativeResize(ct_meas, ct_jacob);
    R_big.conservativeResize(ct_meas, ct_meas);


    // 5. Perform measurement compression
    UpdateUtility::MeasurementCompressInplace(Hx_big, res_big, R_big);

    if (Hx_big.rows() < 1)
    {
        return;
    }

    // 6. With all good features update the state
    StateUtils::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);

    StateUtils::SynchronizePlaneMap(state, toUpdateMapPlane);
    state->SetStatusDebug("Successfully update at\n" + std::to_string(state->timestamp_));
}

void UpdatePlane::CoPlanarUpdate(std::shared_ptr<State> state, const std::vector<std::pair<int, std::vector<int>>> coplane_pos_ids)
{
    // Return if no features
    if(coplane_pos_ids.empty()) return;

    std::cout << "======== Co-Plane Corners ========" << std::endl;

    int plane_size = coplane_pos_ids.size();
    int corner_size = state->featuresPosition_.size();
    // Calculate the max possible measurement size
    size_t max_meas_size =  1 * plane_size * corner_size;
    // Calculate max possible state size (i.e. the size of our covariance)
    int plane_jacob = 3;  // free plane
    size_t max_hx_size = plane_jacob * plane_size + 3 * corner_size;

    // Large Jacobian and residual of *all* features for this update
    Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
    Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
    Eigen::MatrixXd R_big = Eigen::MatrixXd::Zero(max_meas_size, max_meas_size);

    std::unordered_map<std::shared_ptr<BasicType>, size_t> Hx_mapping;
    std::vector<std::shared_ptr<BasicType>> Hx_order_big;
    size_t ct_jacob = 0;
    size_t ct_meas = 0;

    std::vector<int> toUpdateMapPlane, toUpdateCorner;

    /// 以平面分组：每个平面有许多corners作为观测
    for(int i = 0; i < coplane_pos_ids.size(); i++)
    {
        int plane_map_id = coplane_pos_ids[i].first;
        auto plane = state->graphMap_->GetPlane(plane_map_id);

        std::vector<int> pose_ids = coplane_pos_ids[i].second;

        /// 达到一定观测数量才开始用于更新
        if(pose_ids.size() < 2)
            continue;

        Eigen::MatrixXd H_f;  // plane
        Eigen::MatrixXd H_x;  // positions
        Eigen::VectorXd res;  // 1
        Eigen::MatrixXd R;
        std::vector<std::shared_ptr<BasicType> > Hx_order;  // 1 plane

        std::shared_ptr<BasicType> feat;
        if(plane->dir_id_ != -1)  // struct plane（1dof)
        {
            std::shared_ptr<StructurePlane> planeFeat = state->featuresStructurePlane_.at(plane_map_id);
            std::cout << "[CoPlanar Update] coplanar update for structure plane [" << plane_map_id << "]" << std::endl;
            GetCoPlanarStructureJacobianFull(state, plane, pose_ids, H_f, H_x, res, Hx_order, R);  // Get the Jacobian for this feature
            feat = planeFeat;
        }
        else  // free plane（3dof)
        {
            std::shared_ptr<PlaneCP> planeFeat = state->featuresPlane_.at(plane_map_id);
            std::cout << "[CoPlanar Update] coplanar update for free plane [" << plane_map_id << "]" << std::endl;
            GetCoPlanarFreeJacobianFull(state, plane, pose_ids, H_f, H_x, res, Hx_order, R);  // Get the Jacobian for this feature
            feat = planeFeat;
        }

        /// Place Jacobians in one big Jacobian, since the landmark is already in our state vector
        Eigen::MatrixXd H_xf = H_x;
        std::vector<std::shared_ptr<BasicType>> Hxf_order = Hx_order;

        // Else we have the full feature in our state, so just append it
        H_xf.conservativeResize(H_x.rows(), H_x.cols() + H_f.cols());
        H_xf.block(0, H_x.cols(), H_x.rows(), H_f.cols()) = H_f;
        // Append to our Jacobian order vector
        Hxf_order.push_back(feat);


        /// Chi2 distance check
        Eigen::MatrixXd P_marg = StateUtils::GetMarginalCovariance(state, Hxf_order);
        Eigen::MatrixXd S = H_xf * P_marg * H_xf.transpose();
        S.diagonal() += coplanarNoise_ * Eigen::VectorXd::Ones(S.rows());
        double chi2 = res.dot(S.llt().solve(res));

        // Get our threshold (we precompute up to 500 but handle the case that it is more)
        double chi2_check;  //
        if (res.rows() < chiSquaredTable_.size()) {
            chi2_check = chiSquaredTable_[res.rows()];
        } else {
            continue;
        }

        // Check if we should delete or not
        if(chi2 > chi2_multipler_ * chi2_check)
        {
            // 非正常更新
            std::cout << "[CoPlanar Update] (" << plane->map_id_ << ") chi2 check failed." << chi2 << "/ " << chi2_check << std::endl;
            continue;
        }


        /// We are good!!! Append to our large H vector
        size_t ct_hx = 0;
        for (const auto &var : Hxf_order)
        {
            // Ensure that this variable is in our Jacobian
            if (Hx_mapping.find(var)==Hx_mapping.end()) {
                Hx_mapping.insert({var,ct_jacob});
                Hx_order_big.push_back(var);
                ct_jacob += var->GetSize();
            }
            // Append to our large Jacobian
            Hx_big.block(ct_meas, Hx_mapping[var], H_xf.rows(), var->GetSize()).noalias() = H_xf.block(0, ct_hx, H_xf.rows(),var->GetSize());
            ct_hx += var->GetSize();
        }

        // Append our residual and move forward
        res_big.block(ct_meas,0,res.rows(),1).noalias() = res;
        R_big.block(ct_meas, ct_meas, res.rows(), res.rows()).noalias() = R;
        ct_meas += res.rows();

        toUpdateMapPlane.push_back(plane_map_id);

    }  // feature_vec

    // We have appended all features to our Hx_big, res_big
    // Delete it so we do not reuse information

    // Return if we don't have anything and resize our matrices
    if (ct_meas < 1) {
        return;
    }

    assert(ct_meas <= max_meas_size);
    assert(ct_jacob <= max_hx_size);

    res_big.conservativeResize(ct_meas, 1);
    Hx_big.conservativeResize(ct_meas, ct_jacob);
    R_big.conservativeResize(ct_meas, ct_meas);

    // 5. Perform measurement compression
    UpdateUtility::MeasurementCompressInplace(Hx_big, res_big, R_big);

    if (Hx_big.rows() < 1)
    {
        return;
    }

    // 6. With all good features update the state
    StateUtils::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);

    StateUtils::SynchronizePlaneMap(state, toUpdateMapPlane);
    StateUtils::SynchronizeCornerMap(state, toUpdateCorner);
}


void UpdatePlane::GetFreePlaneJacobianFull(std::shared_ptr<State> state,
                                         std::shared_ptr<PlaneFeature> feature, std::vector<int>& obs_index,
                                         Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                                         Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order, Eigen::MatrixXd& R)  // out
{
    // Total number of measurements for this plane feature
    int total_meas = obs_index.size();
    assert(total_meas > 0);

    // Compute the size of the states involved with this feature
    int total_hx = 0;
    std::unordered_map<std::shared_ptr<BasicType>, size_t> map_hx;

    // IMU-camera
    std::shared_ptr<PoseJPL> calibration = state->calibImuToCam_;
    Eigen::Matrix<double, 3, 3> R_ItoC = calibration->GetRotation();
    Eigen::Matrix<double, 3, 1> p_IinC = calibration->GetPos();

    /// 2 If doing calibration extrinsics
    if (state->parameters_.do_calib_camera_extrinsic)
    {
        if (map_hx.find(calibration) == map_hx.end())
        {
            map_hx.insert({calibration, total_hx});
            x_order.push_back(calibration);
            total_hx += calibration->GetSize();
        }
    }

    /// 3 Loop through all measurements for this specific plane
    for (size_t m = 0; m < obs_index.size(); m++)
    {
        int ind = obs_index[m];
        double timestamp = feature->observations_[ind]->timestamp_;

        // Add this clone if it is not added already
        std::shared_ptr<PoseJPL> clone_Ci = state->clonesImu_.at(timestamp);
        if (map_hx.find(clone_Ci) == map_hx.end()) {
            map_hx.insert({clone_Ci, total_hx});
            x_order.push_back(clone_Ci);
            total_hx += clone_Ci->GetSize();
        }
    }

    /// 4
    int map_id = feature->map_id_;
    std::shared_ptr<PlaneCP> plane = state->featuresPlane_.at(map_id);

    Eigen::MatrixXd plane_G = plane->GetValue();
    double d_G = plane_G.norm();
    Eigen::Matrix<double,3 ,1> n_G = plane_G / plane_G.norm();

    // Allocate our residual and Jacobians
    int c = 0;
    int jacobsize = 3;
    res = Eigen::VectorXd::Zero(3 * total_meas);
    H_f = Eigen::MatrixXd::Zero(3 * total_meas, jacobsize);
    H_x = Eigen::MatrixXd::Zero(3 * total_meas, total_hx);
    R = Eigen::MatrixXd::Zero(3 * total_meas, 3 * total_meas);

    // Loop through all measurements for this specific camera
    for (size_t m = 0; m < obs_index.size(); m++)
    {
        int obs_id = obs_index[m];
        double timestamp = feature->observations_[obs_id]->timestamp_;

        std::shared_ptr<PoseJPL> clone_Ii = state->clonesImu_.at(timestamp);
        Eigen::Matrix<double,3,3> R_GtoIi = clone_Ii->GetRotation();
        Eigen::Matrix<double,3,1> p_IiinG = clone_Ii->GetPos();

        Eigen::Matrix<double,3,3> R_GtoCi = R_ItoC * R_GtoIi;
        Eigen::Matrix<double,3,1> p_CiinG = p_IiinG - R_GtoCi.transpose() * p_IinC;

        std::shared_ptr<PlaneObservation> obs = feature->observations_[obs_id];

        /// 1 res R
        Eigen::Matrix<double,3 ,1> m_plane = obs->coeff_->normal_ / obs->coeff_->normal_.norm() * obs->coeff_->d_;
        Eigen::Matrix<double,3 ,1> z_plane = (R_GtoCi * n_G) * (d_G - p_CiinG.transpose() * n_G);
        res.block(3 * c, 0, 3, 1) = m_plane - z_plane;
        R.block(3 * c, 3 * c, 3, 3).noalias() = obs->R_;

        /// 2 calibration
        if(state->parameters_.do_calib_camera_extrinsic)
        {
            Eigen::Matrix<double,3 ,3> HR = QuatUtils::SkewX(R_ItoC * R_GtoIi * (n_G * d_G - n_G * (p_IiinG.transpose() - p_IinC.transpose() * R_ItoC * R_GtoIi) * n_G)) +
                    R_ItoC * R_GtoIi * n_G * p_IinC.transpose() * QuatUtils::SkewX(R_ItoC * R_GtoIi * n_G);
            Eigen::Matrix<double,3 ,3> Hp = R_ItoC * R_GtoIi * n_G * n_G.transpose() * R_GtoIi.transpose() * R_ItoC.transpose();
            Eigen::Matrix<double, 3, 6> dpc_dcalib = Eigen::Matrix<double,3,6>::Zero();
            dpc_dcalib.block(0, 0, 3, 3).noalias() = HR;
            dpc_dcalib.block(0, 3, 3, 3).noalias() = Hp;
            H_x.block(3 * c, map_hx[calibration], 3, calibration->GetSize()).noalias() = dpc_dcalib;
        }

        /// 3 clone
        Eigen::Matrix<double,3,3> dpc_dRgi = R_ItoC * QuatUtils::SkewX(R_GtoIi * n_G * ( d_G - (p_IiinG.transpose() - p_IinC.transpose() * R_ItoC * R_GtoIi) * n_G)) +
                R_ItoC * R_GtoIi * n_G * p_IinC.transpose() * R_ItoC * QuatUtils::SkewX(R_GtoIi * n_G);
        Eigen::Matrix<double,3,3> dpc_dpig = R_ItoC * R_GtoIi * n_G * ( - n_G.transpose());
        Eigen::Matrix<double,3,6> dpc_dclone = Eigen::Matrix<double,3,6>::Zero();
        dpc_dclone.block(0,0,3,3).noalias() = dpc_dRgi;
        dpc_dclone.block(0,3,3,3).noalias() = dpc_dpig;
        H_x.block(3 * c, map_hx[clone_Ii], 3, clone_Ii->GetSize()).noalias() = dpc_dclone;

        /// 4 feature
        Eigen::Matrix<double,3, 4> Hnd = Eigen::Matrix<double,3,4>::Zero();
        Eigen::Matrix<double,3 ,3> Hn = R_ItoC * R_GtoIi * ( d_G - p_IiinG.transpose() * n_G + p_IinC.transpose() * R_ItoC * R_GtoIi * n_G ) +
                                        R_ItoC * R_GtoIi * (- n_G * p_IiinG.transpose() + n_G * p_IinC.transpose() * R_ItoC * R_GtoIi);
        Eigen::Matrix<double,3 ,1> Hd = R_ItoC * R_GtoIi * n_G;
        Hnd.block(0, 0, 3, 3).noalias() = Hn;
        Hnd.block(0, 3, 3, 1).noalias() = Hd;

        Eigen::Matrix<double,4, 3> Hplane = Eigen::Matrix<double,4,3>::Zero();
        Hplane.block(0, 0, 3, 3).noalias() = 1 / d_G * (Eigen::Matrix<double,3,3>::Identity() - n_G * n_G.transpose());
        Hplane.block(3, 0, 1, 3).noalias() = n_G.transpose();
        H_f.block(3 * c, 0, 3, 3).noalias() = Hnd * Hplane;

        // Move the Jacobian and residual index forward
        c++;
    }

    res.conservativeResize(c, 1);
    H_x.conservativeResize(c, total_hx);
    H_f.conservativeResize(c, jacobsize);
    R.conservativeResize(c, c);
}



void UpdatePlane::UpdateCorners(std::shared_ptr<State> state, std::vector<std::shared_ptr<CornerObservation>>& feature_vec)
{
    // Return if no features
    if(feature_vec.empty()) {
        return;
    }

    std::cout << "======== Update Corners ========" << std::endl;
    std::cout << "[Update Corners] Feature size: " << feature_vec.size() << std::endl;

    // 0. Get all timestamps our clones are at (and thus valid measurement times)
    std::vector<double> clonetimes;
    for (const auto& clone_imu : state->clonesImu_) {
        clonetimes.emplace_back(clone_imu.first);
    }

    int size = 6;  // the freedom of 'Corner'
    // Calculate the max possible measurement size
    size_t max_meas_size =  size * feature_vec.size() * (clonetimes.size() + 1);
    // Calculate max possible state size (i.e. the size of our covariance)
    size_t max_hx_size = state->MaxCovarianceSize();

    // Large Jacobian and residual of *all* features for this update
    Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
    Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
    Eigen::MatrixXd R_big = Eigen::MatrixXd::Zero(max_meas_size, max_meas_size);

    std::unordered_map<std::shared_ptr<BasicType>, size_t> Hx_mapping;
    std::vector<std::shared_ptr<BasicType>> Hx_order_big;
    size_t ct_jacob = 0;
    size_t ct_meas = 0;

    std::vector<int> toUpdateCorner;
    for(int i = 0; i < feature_vec.size(); i++)
    {
        int map_id = feature_vec[i]->map_id_;
        int rot_map_id = feature_vec[i]->rot_map_id_;
        int pos_map_id = feature_vec[i]->pos_map_id_;

        std::shared_ptr<Rotation> rotFeat = state->featuresRotation_.at(rot_map_id);
        std::shared_ptr<Position> posFeat = state->featuresPosition_.at(pos_map_id);

        std::cout << "[Update Corners] update for corner [" << map_id << "](" << rot_map_id << ", " << pos_map_id << ") ";

        /// Get valid measurements
        std::shared_ptr<CornerFeature> corner_feature = state->graphMap_->cornersIdLookup_[map_id];
        std::vector<int> valid_indexs;
        corner_feature->GetValidMeasurements(clonetimes, valid_indexs);
        std::cout << "Observation size = " << valid_indexs.size() << std::endl;
        if(valid_indexs.size() < corner_min_measurements_)
            continue;

        /// Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
        Eigen::MatrixXd H_f;
        Eigen::MatrixXd H_x;
        Eigen::VectorXd res;
        Eigen::MatrixXd R;
        std::vector<std::shared_ptr<BasicType> > Hx_order;

        // Get the Jacobian for this feature
        GetCornerJacobianFull(state, corner_feature, valid_indexs, H_f, H_x, res, Hx_order, R);

        /// Place Jacobians in one big Jacobian, since the landmark is already in our state vector
        Eigen::MatrixXd H_xf = H_x;
        std::vector<std::shared_ptr<BasicType>> Hxf_order = Hx_order;
        if(corner_feature->is_fixed_)
        {
            std::cout << "(fix corner landmark!)" << std::endl;
        }
        else
        {
            H_xf.conservativeResize(H_x.rows(), H_x.cols() + H_f.cols());
            H_xf.block(0, H_x.cols(), H_x.rows(), H_f.cols()) = H_f;
            // Append to our Jacobian order vector
            Hxf_order.push_back(rotFeat);
            Hxf_order.push_back(posFeat);
            std::cout << "(update corner landmark!)" << std::endl;
        }


        /// Chi2 distance check
        Eigen::MatrixXd P_marg = StateUtils::GetMarginalCovariance(state, Hxf_order);
        Eigen::MatrixXd S = H_xf * P_marg * H_xf.transpose();
        S += R;
//        S.diagonal() += planeNoise_ * Eigen::VectorXd::Ones(S.rows());
        double chi2 = res.dot(S.llt().solve(res));

        // Get our threshold (we precompute up to 500 but handle the case that it is more)
        double chi2_check;  //
        if (res.rows() < chiSquaredTable_.size()) {
            chi2_check = chiSquaredTable_[res.rows()];
        } else {
            continue;
        }

//        double noise = R.trace();
        // Check if we should delete or not
        if(chi2 > chi2_multipler_ * chi2_check)
        {
            // 非正常更新
            std::cout << "[Update] [" << feature_vec[i]->map_id_ << "] chi2 check failed." << chi2 << "/ " << chi2_check << std::endl;
            continue;
        }


        /// We are good!!! Append to our large H vector
        size_t ct_hx = 0;
        for (const auto &var : Hxf_order)
        {
            // Ensure that this variable is in our Jacobian
            if (Hx_mapping.find(var)==Hx_mapping.end()) {
                Hx_mapping.insert({var,ct_jacob});
                Hx_order_big.push_back(var);
                ct_jacob += var->GetSize();
            }
            // Append to our large Jacobian
            Hx_big.block(ct_meas, Hx_mapping[var], H_xf.rows(), var->GetSize()).noalias() = H_xf.block(0, ct_hx, H_xf.rows(),var->GetSize());
            ct_hx += var->GetSize();
        }

        // Append our residual and move forward
        res_big.block(ct_meas,0,res.rows(),1).noalias() = res;
        R_big.block(ct_meas, ct_meas, res.rows(), res.rows()).noalias() = R;
        ct_meas += res.rows();

        toUpdateCorner.push_back(map_id);
    }  // feature_vec

    // We have appended all features to our Hx_big, res_big
    // Delete it so we do not reuse information

    // Return if we don't have anything and resize our matrices
    if (ct_meas < 1) {
        return;
    }

    assert(ct_meas <= max_meas_size);
    assert(ct_jacob <= max_hx_size);

    res_big.conservativeResize(ct_meas, 1);
    Hx_big.conservativeResize(ct_meas, ct_jacob);
    R_big.conservativeResize(ct_meas, ct_meas);

    // 5. Perform measurement compression
    UpdateUtility::MeasurementCompressInplace(Hx_big, res_big, R_big);

    if (Hx_big.rows() < 1)
    {
        return;
    }

    // 6. With all good features update the state
    StateUtils::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);

    StateUtils::SynchronizeCornerMap(state, toUpdateCorner);
    this_use_corner_update_ = true;
}


void UpdatePlane:: GetCornerJacobianFull(std::shared_ptr<State> state,
                                        std::shared_ptr<CornerFeature> corner_feature, std::vector<int>& obs_index,
                                        Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                                        Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order, Eigen::MatrixXd& R)  // out
{
    // Total number of measurements for this corner feature
    int total_meas = obs_index.size();
    assert(total_meas > 0);

    // Compute the size of the states involved with this feature
    int total_hx = 0;
    std::unordered_map<std::shared_ptr<BasicType>, size_t> map_hx;

    // IMU-camera
    std::shared_ptr<PoseJPL> calibration = state->calibImuToCam_;
    Eigen::Matrix<double, 3, 3> R_ItoC = calibration->GetRotation();
    Eigen::Matrix<double, 3, 1> p_IinC = calibration->GetPos();

    /// If doing calibration extrinsics
    if (state->parameters_.do_calib_camera_extrinsic)
    {
        if (map_hx.find(calibration) == map_hx.end())
        {
            map_hx.insert({calibration, total_hx});
            x_order.push_back(calibration);
            total_hx += calibration->GetSize();
        }
    }

    /// Loop through all measurements for this specific corner
    for (size_t i = 0; i < total_meas; i++)
    {
        int idx = obs_index[i];
        auto corner_obs = corner_feature->cornersObs_[idx];
        double timestamp = corner_obs->timestamp_;

        // Add this clone if it is not added already
        std::shared_ptr<PoseJPL> clone_Ci = state->clonesImu_.at(timestamp);
        if (map_hx.find(clone_Ci) == map_hx.end()) {
            map_hx.insert({clone_Ci, total_hx});
            x_order.push_back(clone_Ci);
            total_hx += clone_Ci->GetSize();
        }
    }

    int rot_id = corner_feature->rot_id_;
    std::shared_ptr<Rotation> rot_landmark = state->featuresRotation_.at(rot_id);
    int pos_id = corner_feature->pos_id_;
    std::shared_ptr<Position> pos_landmark = state->featuresPosition_.at(pos_id);

    Eigen::Matrix<double, 3, 3> R_MtoG = rot_landmark->GetRotation();
    Eigen::Matrix<double, 3, 1> p_MinG = pos_landmark->GetValue();

    // Allocate our residual and Jacobians
    int c = 0;
    int jacobsize = 6;
    int meas_size = 6;
    res = Eigen::VectorXd::Zero(meas_size * total_meas);
    H_f = Eigen::MatrixXd::Zero(meas_size * total_meas, jacobsize);
    H_x = Eigen::MatrixXd::Zero(meas_size * total_meas, total_hx);
    R = Eigen::MatrixXd::Zero(meas_size * total_meas, meas_size * total_meas);

    // Loop through all measurements for this specific camera
    for (size_t i = 0; i < obs_index.size(); i++)
    {
        int idx = obs_index[i];
        auto corner_obs = corner_feature->cornersObs_[idx];

        double timestamp = corner_obs->timestamp_;

        std::shared_ptr<PoseJPL> clone_Ii = state->clonesImu_.at(timestamp);
        Eigen::Matrix<double,3,3> R_GtoIi = clone_Ii->GetRotation();
        Eigen::Matrix<double,3,1> p_IiinG = clone_Ii->GetPos();

//        Eigen::Matrix<double,3,3> R_GtoCi = R_ItoC * R_GtoIi;
//        Eigen::Matrix<double,3,1> p_CiinG = p_IiinG - R_GtoCi.transpose() * p_IinC;
//        Eigen::Matrix<double,3,1> p_GinCi = - R_GtoCi * p_CiinG;

        // orient
        {
//            Eigen::Matrix<double, 4, 1> q_GtoCi = QuatUtils::Rot2Quat(R_GtoCi);
//            Eigen::Matrix<double, 4, 1> q_ItoC = QuatUtils::Rot2Quat(R_ItoC);
//            Eigen::Matrix<double, 4, 1> q_GtoIi = QuatUtils::Rot2Quat(R_GtoIi);
//            Eigen::Matrix<double, 4, 1> q_MtoG = QuatUtils::Rot2Quat(R_MtoG);

//            Eigen::Matrix<double,4,1> m_q_MtoC = QuatUtils::Rot2Quat(m_R_MtoC);
//            Eigen::Matrix<double,4,1> z_q_MtoC = QuatUtils::Rot2Quat(z_R_MtoC);
//            Eigen::Matrix<double,4,1> z_q_MtoC_inv = QuatUtils::Inv(z_q_MtoC);
//            Eigen::Matrix<double,4,1> m_q_MtoC_inv = QuatUtils::Inv(m_q_MtoC);
//            Eigen::Matrix<double,4,1> delta_q2 = QuatUtils::QuatMultiply(m_q_MtoC, z_q_MtoC_inv);
//            std::cout << "delta1: " << delta_q2 << std::endl;
//            std::cout << "delta2: " << delta_q << std::endl;

            /// 1 res & R
            Eigen::Matrix<double,3,3> m_R_MtoC = corner_obs->R_M0toC_ * corner_obs->R_MtoM0_;
            Eigen::Matrix<double,3,3> z_R_MtoC = R_ItoC * R_GtoIi * R_MtoG; // 观测方程：R_MtoC = R_ItoC * R_GtoI * R_MtoG
            Eigen::Matrix<double,3,3> dR = m_R_MtoC * z_R_MtoC.transpose();
            Eigen::Matrix<double,4,1> delta_q = QuatUtils::Rot2Quat(dR);
            Eigen::Matrix<double,3,1> delta_Phi = 2 * delta_q.block(0,0,3,1);
            res.block(c, 0, 3, 1) = delta_Phi;  // delta Phi
            std::cout << "res q: " << delta_q.transpose() << std::endl;

//            std::cout << QuatUtils::QuatMultiply(QuatUtils::Rot2Quat(R_ItoC),QuatUtils::QuatMultiply(QuatUtils::Rot2Quat(R_GtoIi),QuatUtils::Rot2Quat(R_MtoG))) << std::endl;
//            std::cout << QuatUtils::Rot2Quat(z_R_MtoC) << std::endl;
            /// 2 calibration
            if(state->parameters_.do_calib_camera_extrinsic)
            {
                Eigen::Matrix<double, 3, 6> dqmc_dcalib = Eigen::Matrix<double,3,6>::Zero();
                Eigen::Matrix<double, 4, 4> dqm_dcalibq = QuatUtils::QrMatrix(QuatUtils::Rot2Quat(R_GtoIi * R_MtoG));  // first
//                dqfc_dcalib.block(0, 0, 3, 3).noalias() = Eigen::Matrix<double,3,3>::Identity();
                dqmc_dcalib.block(0, 0, 3, 3).noalias() = dqm_dcalibq.block(0,0,3,3);
                dqmc_dcalib.block(0, 3, 3, 3).noalias() = Eigen::Matrix<double,3,3>::Zero();
                H_x.block(c, map_hx[calibration], 3, calibration->GetSize()).noalias() = dqmc_dcalib;
                std::cout << "d calib: " << std::endl << dqmc_dcalib << std::endl;
            }

            /// 3 clone
            Eigen::Matrix<double,3,6> dqmc_dclone = Eigen::Matrix<double,3,6>::Zero();
//            Eigen::Matrix<double, 4, 4> dqclone = QuatUtils::QrMatrix(QuatUtils::Inv(q_ItoC)) * QuatUtils::QlMatrix(q_ItoC);
            Eigen::Matrix<double, 4, 4> dqm_dcloneq = QuatUtils::QlMatrix(QuatUtils::Rot2Quat(R_ItoC)) * QuatUtils::QrMatrix(QuatUtils::Rot2Quat(R_MtoG));  // middle
            dqmc_dclone.block(0,0,3,3).noalias() = dqm_dcloneq.block(0,0,3,3);
            dqmc_dclone.block(0,3,3,3).noalias() = Eigen::Matrix<double,3,3>::Zero();
            H_x.block(c, map_hx[clone_Ii], 3, clone_Ii->GetSize()).noalias() = dqmc_dclone;
            std::cout << "d clone: " << std::endl << dqmc_dclone << std::endl;

            /// 4 feature
//            Eigen::Matrix<double, 4, 4> dqf = QuatUtils::QrMatrix(QuatUtils::Inv(q_GtoCi)) * QuatUtils::QlMatrix(q_GtoCi);
            Eigen::Matrix<double, 4, 4> dqmc_dqmg = QuatUtils::QlMatrix(QuatUtils::Rot2Quat(R_ItoC * R_GtoIi));  // last
            H_f.block(c, 0, 3, 3).noalias() = dqmc_dqmg.block(0,0,3,3);
            std::cout << "d rotation: " << std::endl << dqmc_dqmg.block(0,0,3,3) << std::endl;

            // Move the Jacobian and residual index forward
            c+=3;
        }

        // pos
        {
//            Eigen::Matrix<double,3,1> p_MinIi = R_GtoIi * (p_MinG - p_IiinG);
//            Eigen::Matrix<double,3,1> p_MinCi = R_ItoC * p_MinIi + p_IinC;
            /// 1 res & R
            Eigen::Matrix<double,3 ,1> m_corner = corner_obs->position_;
            Eigen::Matrix<double,3 ,1> z_corner = R_ItoC * R_GtoIi * (p_MinG  - p_IiinG) + p_IinC;
            res.block(c, 0, 3, 1) = m_corner - z_corner;
//            R.block(c, c, 3, 3).noalias() = corner_obs->pos_R_;
            R.block(c, c, 3, 3).noalias() = corner_position_noise * Eigen::Matrix3d::Identity();
            std::cout << "res p: " << (m_corner - z_corner).transpose() << std::endl;
            std::cout << "R p: " << std::endl << corner_obs->pos_R_ << std::endl;

            /// 2 calibration
            if(state->parameters_.do_calib_camera_extrinsic)
            {
                Eigen::Matrix<double, 3, 6> dpmc_dcalib = Eigen::Matrix<double,3,6>::Zero();
                dpmc_dcalib.block(0, 0, 3, 3).noalias() = QuatUtils::SkewX(R_ItoC * R_GtoIi * (p_MinG - p_IiinG));
                dpmc_dcalib.block(0, 3, 3, 3).noalias() = Eigen::Matrix<double,3,3>::Identity();
                H_x.block(c, map_hx[calibration], 3, calibration->GetSize()).noalias() = dpmc_dcalib;
            }

            /// 3 clone
            Eigen::Matrix<double,3,6> dpmc_dclone = Eigen::Matrix<double,3,6>::Zero();
            dpmc_dclone.block(0,0,3,3).noalias() = R_ItoC * QuatUtils::SkewX(R_GtoIi * (p_MinG - p_IiinG));
            dpmc_dclone.block(0,3,3,3) = -R_ItoC * R_GtoIi;
            H_x.block(c, map_hx[clone_Ii], 3, clone_Ii->GetSize()).noalias() = dpmc_dclone;

            /// 4 feature
            H_f.block(c, 3, 3, 3).noalias() = R_ItoC * R_GtoIi;

            c+=3;
        }

    }

    res.conservativeResize(c, 1);
    H_x.conservativeResize(c, total_hx);
    H_f.conservativeResize(c, jacobsize);
    R.conservativeResize(c, c);
}


void UpdatePlane::GetCoPlanarFreeJacobianFull(std::shared_ptr<State> state,
                                           std::shared_ptr<PlaneFeature> feature,
                                           std::vector<int>& pos_ids,
                                           Eigen::MatrixXd &H_f, Eigen::MatrixXd &H_x,  // out
                                           Eigen::VectorXd &res, std::vector<std::shared_ptr<BasicType>> &x_order, Eigen::MatrixXd& R)  // out
{

    // Total number of measurements for this plane feature
    int total_meas = pos_ids.size();
    assert(total_meas > 0);

    // Compute the size of the states involved with this feature
    int total_hx = 0;
    std::unordered_map<std::shared_ptr<BasicType>, size_t> map_hx;

    /// 3 Loop through all measurements for this specific plane
    for (size_t m = 0; m < pos_ids.size(); m++)
    {
        int pos_id = pos_ids[m];
        std::shared_ptr<Position> pos_landmark = state->featuresPosition_.at(pos_id);

        if (map_hx.find(pos_landmark) == map_hx.end()) {
            map_hx.insert({pos_landmark, total_hx});
            x_order.push_back(pos_landmark);
            total_hx += pos_landmark->GetSize();
        }
    }

    int plane_map_id = feature->map_id_;
    std::shared_ptr<PlaneCP> plane = state->featuresPlane_.at(plane_map_id);

    Eigen::MatrixXd plane_G = plane->GetValue();
    double d_G_double = plane_G.norm();
    Eigen::Matrix<double,1,1> d_G; d_G << d_G_double;
    Eigen::Matrix<double,3,1> n_G = plane_G / plane_G.norm();

    // Allocate our residual and Jacobians
    int c = 0;
    int jacobsize = 3;
    res = Eigen::VectorXd::Zero(1 * total_meas);
    H_f = Eigen::MatrixXd::Zero(1 * total_meas, jacobsize);
    H_x = Eigen::MatrixXd::Zero(1 * total_meas, total_hx);
    R = Eigen::MatrixXd::Zero(1 * total_meas, 1 * total_meas);

    // Loop through all measurements for this specific camera
    for (size_t m = 0; m < pos_ids.size(); m++)
    {
        int pos_id = pos_ids[m];
        std::shared_ptr<Position> pos_landmark = state->featuresPosition_.at(pos_id);
        Eigen::Matrix<double,3,1> position = pos_landmark->GetValue();

        /// 1 res R
        Eigen::Matrix<double,1 ,1> m_dis = Eigen::Matrix<double,1 ,1>::Zero();
        Eigen::Matrix<double,1 ,1> z_dis = position.transpose() * n_G - d_G;
        res.block(c, 0, 1, 1) = m_dis - z_dis;
        R.block(c, c, 1, 1).noalias() = coplanarNoise_ * Eigen::Matrix<double,1 ,1>::Identity();

        /// 2 position
        Eigen::Matrix<double,1,3> dz_p = n_G.transpose();
        H_x.block(c, map_hx[pos_landmark], 1, pos_landmark->GetSize()).noalias() = dz_p;

        /// 3 plane
        Eigen::Matrix<double,1, 4> Hnd = Eigen::Matrix<double,1,4>::Zero();
        Eigen::Matrix<double,1 ,3> Hn = position.transpose();
        Eigen::Matrix<double,1 ,1> Hd; Hd << -1;
        Hnd.block(0, 0, 1, 3).noalias() = Hn;
        Hnd.block(0, 3, 1, 1).noalias() = Hd;

        Eigen::Matrix<double,4, 3> Hplane = Eigen::Matrix<double,4,3>::Zero();
        Hplane.block(0, 0, 3, 3).noalias() = 1 / d_G_double * (Eigen::Matrix<double,3,3>::Identity() - n_G * n_G.transpose());
        Hplane.block(3, 0, 1, 3).noalias() = n_G.transpose();
        H_f.block(c, 0, 1, 3).noalias() = Hnd * Hplane;

        // Move the Jacobian and residual index forward
        c++;
    }

    res.conservativeResize(c, 1);
    H_x.conservativeResize(c, total_hx);
    H_f.conservativeResize(c, jacobsize);
    R.conservativeResize(c, c);
}