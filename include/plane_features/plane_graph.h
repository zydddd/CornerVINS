//
// Created by zyd on 24-1-20.
//

#ifndef RGBD_PLANE_VIO_PLANE_GRAPH_H
#define RGBD_PLANE_VIO_PLANE_GRAPH_H

#include "plane_feature.h"

extern bool upspeed;

namespace Rvg{

    /// edge
    enum Relationship{NONE, PARALLEL, SEPERATION, INTERSECTION};  //
    class PlaneRelation
    {
    public:
        Relationship relation_ = NONE;
        double distance_ = 100;  //
        double overlap_ = -5.0;  //
        double angle_ = 100;  // 0-pi/2
        std::pair<int, int> nodes_;  // id, id
    };


    class PlaneGraph
    {
    public:
        std::vector<std::shared_ptr<PlaneRelation>> edges_;
        std::vector<std::vector<int>> edgesMatrix_;  //
        std::unordered_map<int, int> index_;  // map map_id to current id
        int n = 0;  //
        int m = 0; //


        double angle_thres = 10;  //

        std::vector<std::shared_ptr<PlaneObservation>> ConvertToWorld(const std::vector<std::shared_ptr<PlaneObservation>>& planes);
    };


    class ObservationGraph: public PlaneGraph
    {
    public:

        ObservationGraph();
        ObservationGraph(std::vector<std::shared_ptr<PlaneObservation>>& obs);
        ~ObservationGraph();

        std::shared_ptr<PlaneRelation> ComputeEdge(const std::shared_ptr<PlaneObservation> p1, const std::shared_ptr<PlaneObservation> p2);

        std::vector<std::shared_ptr<PlaneObservation>> vertices_;

    };

    class FeatureGraph: public PlaneGraph
    {
    public:
        FeatureGraph();
        ~FeatureGraph();

        void AppendNode(const int mapId, std::shared_ptr<PlaneObservation>& obs);
        void AppendEdge(const int mapId1, const int mapId2);
        void UpdateNode(const int nodeId, Eigen::Vector3d& plane);
        void UpdateEdge(const int nodeId1, const int nodeId2);
        std::shared_ptr<FeatureGraph> Marginalize();
        std::shared_ptr<PlaneRelation> ComputeEdge(const std::shared_ptr<PlaneFeature> p1, const std::shared_ptr<PlaneFeature> p2);
        void AppendNewPlaneObservations(const std::vector<std::shared_ptr<PlaneObservation>>& plane_obs);
        void AppendNewCornerObservations(const std::vector<std::shared_ptr<CornerObservation>>& corner_obs);
        std::vector<std::shared_ptr<PlaneFeature>> vertices_;

        // TODO merge thres
        double merge_angle_thres = 5;
        double merge_dis_thres = 0.05;
        double merge_overlap_thres = 0.0;

        std::unordered_set<int> local_map_ids_;  //


        /// Lookup array that allow use to query based on ID
        std::unordered_map<size_t, std::shared_ptr<CornerFeature>> cornersIdLookup_;
        std::unordered_map<size_t, Eigen::Matrix3d> rotationsIdLookup_;
        std::unordered_map<size_t, Eigen::Vector3d> positionsIdLookup_;
        std::unordered_map<size_t, std::shared_ptr<DirectionFeature>> directionsIdLookup_;  //

        const Eigen::Vector3d& GetCornerPosition(const int corner_id);
        const Eigen::Matrix3d& GetCornerRotation(const int corner_id);
        const std::shared_ptr<PlaneFeature> GetPlane(const int map_id);

    };

}

#endif //RGBD_PLANE_VIO_PLANE_GRAPH_H
