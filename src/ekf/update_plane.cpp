//
// Created by zyd on 23-1-2.
//

#include "update_plane.h"
#include <iomanip>

using namespace Rvg;

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

    std::unordered_map<std::shared_ptr<BasicType>, size_t> Hx_mapping;  //
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
        std::vector<std::shared_ptr<BasicType> > Hx_order;  //

        int map_id = feature_vec[i]->map_id_;
        std::shared_ptr<PlaneFeature> plane = state->graphMap_->GetPlane(map_id);

        /// Get valid measurements 达到一定观测数量才开始用于更新
        std::vector<int> timestamp_index;
        plane->GetValidMeasurements(clonetimes, timestamp_index);
        if(timestamp_index.size() < min_measurements_)
            continue;


        std::shared_ptr<BasicType> feat;
        if(plane->dir_id_ != -1)  //
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
        else  //
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
            //
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
//        S.diagonal() += planeNoise_ * Eigen::VectorXd::Ones(S.rows());  //
        double chi2 = res.dot(S.llt().solve(res));  //

        // Get our threshold (we precompute up to 500 but handle the case that it is more)
        double chi2_check;  //
        if (res.rows() < chiSquaredTable_.size()) {
            chi2_check = chiSquaredTable_[res.rows()];
        } else {
            continue;
        }

//        double noise = R.trace();
        // Check if we should delete or not
        //

        if(chi2 > chi2_multipler_ * chi2_check)
        {
            //
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

    ///
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
    int c = 0;  // 表示观测序号
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
        /// If we are doing first estimate Jacobians, then overwrite with the first estimates
//        if (state->parameters_.do_fej)
//        {
//            // plane
//            plane_G = plane->GetFej();
//            d_G = plane_G.norm();
//            n_G = plane_G / plane_G.norm();
//
//            // clone
//            R_GtoIi = clone_Ii->GetRotationFej();
//            p_IiinG = clone_Ii->GetPosFej();
//
//            // calibration
//            R_ItoC = calibration->GetRotationFej();
//            p_IinC = calibration->GetPosFej();
//
//            R_GtoCi = R_ItoC * R_GtoIi;
//            p_CiinG = p_IiinG - R_GtoCi.transpose() * p_IinC;
//        }

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

void UpdatePlane::GetFreePlaneJacobianP(std::shared_ptr<State> state,
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
    int c = 0;  // 表示观测序号
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
        /// If we are doing first estimate Jacobians, then overwrite with the first estimates

        /// 2 calibration
        if(state->parameters_.do_calib_camera_extrinsic)
        {
            Eigen::Matrix<double,3 ,3> HR = QuatUtils::SkewX(R_ItoC * R_GtoIi * (n_G * d_G - n_G * (p_IiinG.transpose() - p_IinC.transpose() * R_ItoC * R_GtoIi) * n_G)) +
                                            R_ItoC * R_GtoIi * n_G * p_IinC.transpose() * QuatUtils::SkewX(R_ItoC * R_GtoIi * n_G);
            Eigen::Matrix<double,3 ,3> Hp = R_ItoC * R_GtoIi * n_G * n_G.transpose() * R_GtoIi.transpose() * R_ItoC.transpose();
            Eigen::Matrix<double, 3, 6> dpc_dcalib = Eigen::Matrix<double,3,6>::Zero();
//            dpc_dcalib.block(0, 0, 3, 3).noalias() = HR;
            dpc_dcalib.block(0, 3, 3, 3).noalias() = Hp;
            H_x.block(3 * c, map_hx[calibration], 3, calibration->GetSize()).noalias() = dpc_dcalib;
        }

        /// 3 clone
        Eigen::Matrix<double,3,3> dpc_dRgi = R_ItoC * QuatUtils::SkewX(R_GtoIi * n_G * ( d_G - (p_IiinG.transpose() - p_IinC.transpose() * R_ItoC * R_GtoIi) * n_G)) +
                                             R_ItoC * R_GtoIi * n_G * p_IinC.transpose() * R_ItoC * QuatUtils::SkewX(R_GtoIi * n_G);
        Eigen::Matrix<double,3,3> dpc_dpig = R_ItoC * R_GtoIi * n_G * ( - n_G.transpose());
        Eigen::Matrix<double,3,6> dpc_dclone = Eigen::Matrix<double,3,6>::Zero();
//        dpc_dclone.block(0,0,3,3).noalias() = dpc_dRgi;
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

void UpdatePlane::GetStructurePlaneJacobianFull(std::shared_ptr<State> state,
                                                std::shared_ptr<PlaneFeature> feature,
                                                std::vector<int>& obs_index,
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
        int index = obs_index[m];
        double timestamp = feature->observations_[index]->timestamp_;

        // Add this clone if it is not added already
        std::shared_ptr<PoseJPL> clone_Ci = state->clonesImu_.at(timestamp);
        if (map_hx.find(clone_Ci) == map_hx.end()) {
            map_hx.insert({clone_Ci, total_hx});
            x_order.push_back(clone_Ci);
            total_hx += clone_Ci->GetSize();
        }
    }


    /// 4
    int dir_id = feature->dir_id_;
    std::shared_ptr<DirectionFeature> direction = state->graphMap_->directionsIdLookup_.at(dir_id);
    Eigen::Vector3d n_G = direction->direction_;
    n_G = n_G / n_G.norm();
    std::shared_ptr<StructurePlane> sp = state->featuresStructurePlane_.at(feature->map_id_);
    double d_G = sp->GetValue()(0);

    // Allocate our residual and Jacobians
    int c = 0;  //
    int jacobsize = 1;
    int meassize = 3;
    res = Eigen::VectorXd::Zero(meassize * total_meas);
    H_f = Eigen::MatrixXd::Zero(meassize * total_meas, jacobsize);
    H_x = Eigen::MatrixXd::Zero(meassize * total_meas, total_hx);
    R = Eigen::MatrixXd::Zero(meassize * total_meas, meassize * total_meas);

    // Loop through all measurements for this specific camera
    for (size_t m = 0; m < obs_index.size(); m++)
    {
        int index = obs_index[m];
        double timestamp = feature->observations_[index]->timestamp_;

        std::shared_ptr<PoseJPL> clone_Ii = state->clonesImu_.at(timestamp);
        Eigen::Matrix<double,3,3> R_GtoIi = clone_Ii->GetRotation();
        Eigen::Matrix<double,3,1> p_IiinG = clone_Ii->GetPos();

        Eigen::Matrix<double,3,3> R_GtoCi = R_ItoC * R_GtoIi;
        Eigen::Matrix<double,3,1> p_CiinG = p_IiinG - R_GtoCi.transpose() * p_IinC;

        std::shared_ptr<PlaneObservation> obs = feature->observations_[index];

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
        Eigen::Matrix<double,3 ,1> Hd = R_ItoC * R_GtoIi * n_G;  // 关于d的导数
        H_f.block(3 * c, 0, 3, 1).noalias() = Hd;

        // Move the Jacobian and residual index forward
        c++;
    }

    res.conservativeResize(c, 1);
    H_x.conservativeResize(c, total_hx);
    H_f.conservativeResize(c, jacobsize);
    R.conservativeResize(c, c);
}

void UpdatePlane::GetStructurePlaneJacobianP(std::shared_ptr<State> state,
                                                std::shared_ptr<PlaneFeature> feature,
                                                std::vector<int>& obs_index,
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
        int index = obs_index[m];
        double timestamp = feature->observations_[index]->timestamp_;

        // Add this clone if it is not added already
        std::shared_ptr<PoseJPL> clone_Ci = state->clonesImu_.at(timestamp);
        if (map_hx.find(clone_Ci) == map_hx.end()) {
            map_hx.insert({clone_Ci, total_hx});
            x_order.push_back(clone_Ci);
            total_hx += clone_Ci->GetSize();
        }
    }


    /// 4
    int dir_id = feature->dir_id_;
    std::shared_ptr<DirectionFeature> direction = state->graphMap_->directionsIdLookup_.at(dir_id);
    Eigen::Vector3d n_G = direction->direction_;
    n_G = n_G / n_G.norm();
    std::shared_ptr<StructurePlane> sp = state->featuresStructurePlane_.at(feature->map_id_);
    double d_G = sp->GetValue()(0);

    // Allocate our residual and Jacobians
    int c = 0;  // 表示观测序号
    int jacobsize = 1;
    int meassize = 3;
    res = Eigen::VectorXd::Zero(meassize * total_meas);
    H_f = Eigen::MatrixXd::Zero(meassize * total_meas, jacobsize);
    H_x = Eigen::MatrixXd::Zero(meassize * total_meas, total_hx);
    R = Eigen::MatrixXd::Zero(meassize * total_meas, meassize * total_meas);

    // Loop through all measurements for this specific camera
    for (size_t m = 0; m < obs_index.size(); m++)
    {
        int index = obs_index[m];
        double timestamp = feature->observations_[index]->timestamp_;

        std::shared_ptr<PoseJPL> clone_Ii = state->clonesImu_.at(timestamp);
        Eigen::Matrix<double,3,3> R_GtoIi = clone_Ii->GetRotation();
        Eigen::Matrix<double,3,1> p_IiinG = clone_Ii->GetPos();

        Eigen::Matrix<double,3,3> R_GtoCi = R_ItoC * R_GtoIi;
        Eigen::Matrix<double,3,1> p_CiinG = p_IiinG - R_GtoCi.transpose() * p_IinC;

        std::shared_ptr<PlaneObservation> obs = feature->observations_[index];

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
//            dpc_dcalib.block(0, 0, 3, 3).noalias() = HR;
            dpc_dcalib.block(0, 3, 3, 3).noalias() = Hp;
            H_x.block(3 * c, map_hx[calibration], 3, calibration->GetSize()).noalias() = dpc_dcalib;
        }

        /// 3 clone
        Eigen::Matrix<double,3,3> dpc_dRgi = R_ItoC * QuatUtils::SkewX(R_GtoIi * n_G * ( d_G - (p_IiinG.transpose() - p_IinC.transpose() * R_ItoC * R_GtoIi) * n_G)) +
                                             R_ItoC * R_GtoIi * n_G * p_IinC.transpose() * R_ItoC * QuatUtils::SkewX(R_GtoIi * n_G);
        Eigen::Matrix<double,3,3> dpc_dpig = R_ItoC * R_GtoIi * n_G * ( - n_G.transpose());
        Eigen::Matrix<double,3,6> dpc_dclone = Eigen::Matrix<double,3,6>::Zero();
//        dpc_dclone.block(0,0,3,3).noalias() = dpc_dRgi;
        dpc_dclone.block(0,3,3,3).noalias() = dpc_dpig;
        H_x.block(3 * c, map_hx[clone_Ii], 3, clone_Ii->GetSize()).noalias() = dpc_dclone;


        /// 4 feature
        Eigen::Matrix<double,3 ,1> Hd = R_ItoC * R_GtoIi * n_G;  // 关于d的导数
        H_f.block(3 * c, 0, 3, 1).noalias() = Hd;

        // Move the Jacobian and residual index forward
        c++;
    }

    res.conservativeResize(c, 1);
    H_x.conservativeResize(c, total_hx);
    H_f.conservativeResize(c, jacobsize);
    R.conservativeResize(c, c);
}