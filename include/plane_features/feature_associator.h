//
// Created by zyd on 24-5-23.
//

#ifndef RGBD_PLANE_VIO_FEATURE_ASSOCIATOR_H
#define RGBD_PLANE_VIO_FEATURE_ASSOCIATOR_H

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <unordered_map>

#include "plane_feature.h"
#include "utils.h"
#include "plane_graph.h"

namespace Rvg{
    class FeatureAssociator {
    public:
        FeatureAssociator(double angle, double dis, double overlap, bool use_HM, bool share_rotation);


        bool IsMatch(const std::shared_ptr<PlaneCoeff>& p1, const std::shared_ptr<PlaneCoeff>& p2);
        double MatchScore(const std::shared_ptr<PlaneCoeff>& p1, const std::shared_ptr<PlaneCoeff>& p2);

        void PlaneMatching(std::shared_ptr<FeatureGraph> mapGraph,
                           std::vector<std::shared_ptr<CornerObservation>>& corners,
                           std::vector<std::shared_ptr<PlaneObservation>>& planes,
                           Eigen::Matrix<double, 3, 3>& R_GtoCi,
                           Eigen::Matrix<double, 3, 1>& p_CiinG);

        void HierarchicalMatching(std::shared_ptr<FeatureGraph> mapGraph,
                                  std::vector<std::shared_ptr<CornerObservation>>& corners,
                                  std::vector<std::shared_ptr<PlaneObservation>>& planes,
                                  Eigen::Matrix<double, 3, 3>& R_GtoCi, Eigen::Matrix<double, 3, 1>& p_CiinG);
        void PlaneCornerAssociate(std::shared_ptr<FeatureGraph> mapGraph,
                                                     std::vector<std::shared_ptr<CornerObservation>>& corners,
                                                     std::vector<std::shared_ptr<PlaneObservation>>& planes);

        void CornerMatching(std::shared_ptr<FeatureGraph> mapGraph,
                            std::vector<std::shared_ptr<CornerObservation>>& corners,
                            std::vector<std::shared_ptr<PlaneObservation>>& planes,
                            Eigen::Matrix<double, 3, 3>& R_GtoCi, Eigen::Matrix<double, 3, 1>& p_CiinG);

        bool IsRotationMatch(Eigen::Matrix<double, 3, 3>& R_MtoG, Eigen::Matrix<double, 3, 3>& R_M0toG,
                                Eigen::Matrix<double, 3, 3>& R_MtoM0);

        void UpdateDirectionFeature(std::shared_ptr<FeatureGraph> mapGraph,
                                    const int dir_map_id, Eigen::Vector3d& direction, const int plane_map_id);
        int DoubleCheck(std::shared_ptr<FeatureGraph> mapGraph,
                        Eigen::Vector3d& direction);

        void DirectionMatching(std::shared_ptr<FeatureGraph> mapGraph,
                               std::vector<std::shared_ptr<CornerObservation>>& corners,
                               std::vector<std::shared_ptr<PlaneObservation>>& planes,
                               Eigen::Matrix<double, 3, 3>& R_GtoCi, Eigen::Matrix<double, 3, 1>& p_CiinG);

        std::atomic<int> planeFeatureId;
    private:
        std::ofstream debug_file;

        /// Master ID for this associate (atomic to allow for multi-threading)

        std::atomic<int> cornerFeatureId;
        std::atomic<int> rotFeatureId;
        std::atomic<int> posFeatureId;
        std::atomic<int> directionFeatureId;

        ///
        std::vector<std::vector<int>> R_align;  // R_M1toM0
        std::vector<Eigen::Matrix3d> M_align;  // R_MtoM1


        /// Thresholds for associating planes
        double angle_thres_ = 15.0;
        double dis_thres_ = 0.1;
        double overlap_thres_ = 0.0;
        bool use_hierarchical_matching_ = false;
        bool share_rotation_ = false;


        ///
        double rotation_angle_error_ = 10;
        double position_error_ = 0.3;
        bool use_local_tracking_ = true;

    };
}


#endif //RGBD_PLANE_VIO_FEATURE_ASSOCIATOR_H
