//
// Created by zyd on 23-12-13.
//

#ifndef RGBD_PLANE_VIO_PLANE_GRAPH_TRACKER_H
#define RGBD_PLANE_VIO_PLANE_GRAPH_TRACKER_H

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <unordered_map>

#include "plane_feature.h"
#include "plane_graph.h"

#include "plane_features/FGM/hungarian.hpp"
#include "plane_features/FGM/fgm.hpp"
#include "utils.h"

namespace Rvg {

    class PlaneTracker
    {
    public:
        PlaneTracker(double angle, double dis, double overlap,
                     double drift_angle, double drift_dis, double drift_overlap,
                     double binary_angle, double binary_dis, double binary_overlap,
                     int drift_plane_num);
        ~PlaneTracker();

        double MatchScore(const std::shared_ptr<PlaneCoeff>& p1, const std::shared_ptr<PlaneCoeff>& p2);
        bool IsMatch(const std::shared_ptr<PlaneCoeff>& p1, const std::shared_ptr<PlaneCoeff>& p2);

        std::vector<int> FindMatchForce(const std::shared_ptr<ObservationGraph>& graph,
                                                     const std::shared_ptr<FeatureGraph>& graphmap);


        void ForceMatching(std::shared_ptr<FeatureGraph>& graphMap, std::vector<std::shared_ptr<PlaneObservation>>& obs);

        bool DriftDetection(std::shared_ptr<FeatureGraph>& graphMap, std::vector<std::pair<int,int>>& relocate_pair);

        Eigen::MatrixXd UnaryCheckDrift(const std::shared_ptr<ObservationGraph>& src, const std::shared_ptr<FeatureGraph>& dst);
        bool GraphMatching(const std::shared_ptr<ObservationGraph>& activeGraph,
                           const std::shared_ptr<FeatureGraph>& mapGraph,
                           const Eigen::MatrixXd& Ct,
                           std::vector<int>& assignment);
        bool BinaryCheck(std::shared_ptr<ObservationGraph> activeGraph, int id1, int id2,
                         std::shared_ptr<FeatureGraph> mapGraph, int map_id1, int map_id2);


        void ComputeAffinity(const std::shared_ptr<ObservationGraph>& src, const std::shared_ptr<FeatureGraph>& dst,
                             Eigen::MatrixXd& KP, Eigen::MatrixXd& KQ, Eigen::MatrixXd& K,
                             std::vector<std::pair<int, int>>& E1,
                             std::vector<std::pair<int, int>>& E2);

        Eigen::MatrixXd FGMPlus(const Eigen::MatrixXd& KP, const Eigen::MatrixXd& KQ, const Eigen::MatrixXd& Ct,
                                const std::vector<std::pair<int, int>>& E1,
                                const std::vector<std::pair<int, int>>& E2);

        Eigen::MatrixXd SubFGM(const Eigen::MatrixXd& KP, const Eigen::MatrixXd& KQ, const Eigen::MatrixXd& Ct,
                                     const std::vector<std::pair<int, int>>& E1,
                                     const std::vector<std::pair<int, int>>& E2,
                                     const std::vector<int>& index1,
                                     const std::vector<int>& index2);

        void ComputeSubAffinity(const Eigen::MatrixXd& KP, const Eigen::MatrixXd& KQ,
                                const std::vector<std::pair<int, int>>& E1,
                                const std::vector<std::pair<int, int>>& E2,
                                const std::vector<int>& index1, const std::vector<int>& index2,  //
                                const Eigen::MatrixXd& Ct,
                                std::vector<std::pair<int, int>>& sE1,
                                std::vector<std::pair<int, int>>& sE2,
                                Eigen::MatrixXd& sKP, Eigen::MatrixXd& sKQ, Eigen::MatrixXd& sCt);

        void ConvertEdges(const std::vector<std::pair<int, int>>& edges, Eigen::MatrixXd& G, Eigen::MatrixXd& H);



        bool OverlapCheck(std::shared_ptr<ObservationGraph> activeGraph, std::shared_ptr<FeatureGraph> mapGraph, std::vector<std::pair<int,int>>& relocate_pair);
        bool CheckEdge(std::shared_ptr<PlaneRelation> e1, std::shared_ptr<PlaneRelation> e2);


        /// Master ID for this tracker (atomic to allow for multi-threading)
        std::atomic<int> planeFeatureId;

    private:


        /// Thresholds for associating planes
        double angle_thres_ = 10.0;
        double dis_thres_ = 0.05;
        double overlap_thres_ = 0.0;

        /// Thresholds for drift detection
        double drift_angle_thres_ = 20.0;
        double drift_dis_thres_ = 0.5;
        double drift_overlap_thres_ = 0.0;

        /// Thresholds for binary check
        double binary_angle_thres_ = 10.0;
        double binary_dis_thres_ = 0.5;
        double binary_overlap_thres_ = 5;

        /////
        // plane association
        bool use_local_tracking_ = true;
        ////////////// drift suppression
        bool use_undrift_for_check = true; //
        int consistent_node_num = 2;
        double binary_dis_thres_parall_ = 0.05;
        int local_map_size_ = 3; // local map size
        int drift_plane_num_ = 3; //
        double overall_overlap_thres_ = 0.0;
        double area_ratio = 0.5;
        double overlap_ratio = 0.3;
        double area_thres_ = 1.0;
        std::ofstream debug_file;
        ////////////////////////////////
    };
}

#endif //RGBD_PLANE_VIO_PLANE_GRAPH_TRACKER_H
