//
// Created by zyd on 23-10-23.
//

#include <vector>
#include <iostream>
#include <unordered_map>
#include <Eigen/Eigen>
#include <memory>

#include "utils.h"

#ifndef RGBD_PLANE_VIO_PLANE_FEATURE_H
#define RGBD_PLANE_VIO_PLANE_FEATURE_H

extern bool upspeed;

namespace Rvg
{

    class BorderObservation
    {
    public:
        BorderObservation(int i, int j, std::pair<Eigen::Vector3d, Eigen::Vector3d> ep)
        {
            ids_ = std::make_pair(i,j);
            end_points_ = ep;
        }
        std::pair<Eigen::Vector3d, Eigen::Vector3d> end_points_;
        std::pair<int, int> ids_;
    };

    class BorderFeature
    {
    public:
        BorderFeature(int i, int j, std::pair<Eigen::Vector3d, Eigen::Vector3d> ep)
        {
            ids_ = std::make_pair(i,j);
            end_points_ = ep;
        }
        std::pair<Eigen::Vector3d, Eigen::Vector3d> end_points_;
        std::pair<int, int> ids_;
    };

    class CornerObservation
    {
    public:
        CornerObservation(){};
        CornerObservation(double timestamp, int i, int j, int k,
                          Eigen::Vector3d& position, Eigen::Matrix<double, 3, 3>& bases)
        {
            timestamp_ = timestamp;
            plane_ids_[0] = i;
            plane_ids_[1] = j;
            plane_ids_[2] = k;
            position_ = position;
            R_M0toC_ = bases;
        }


        double timestamp_;
        int plane_ids_[3];  //
        Eigen::Vector3d position_; //
        Eigen::Matrix<double, 3, 3> R_M0toC_;  //

        Eigen::Vector3d position_before; //
        Eigen::Matrix<double, 3, 3> bases_before;  //

        int map_id_ = -1;
        int rot_map_id_ = -1;
        int pos_map_id_ = -1;
        bool is_matched_ = false;
        Eigen::Vector3d line_length_;  //
        Eigen::MatrixXd pos_R_ = 1e-3 * Eigen::MatrixXd::Identity(3,3);  //
        Eigen::Matrix<double,3,3> R_MtoM0_ = Eigen::MatrixXd::Zero(3,3);  //

//        Eigen::Matrix<double,3,3> R_GtoIi_;
//        Eigen::Matrix<double,3,1> p_IiinG_;
//        Eigen::Matrix<double,3,3> R_ItoC_;
//        Eigen::Matrix<double,3,1> p_IinC_;
        Eigen::Matrix<double,3,3> R_GtoC_;
        Eigen::Matrix<double,3,1> p_CinG_;

        void SetPose(Eigen::Matrix<double,3,3> R_GtoI, Eigen::Matrix<double,3,1> p_IinG,
                     Eigen::Matrix<double,3,3> R_ItoC, Eigen::Matrix<double,3,1> p_IinC);
        void SetPose(Eigen::Matrix<double,3,3> R_GtoC, Eigen::Matrix<double,3,1> p_CinG);
    };

    class CornerFeature
    {
    public:
        CornerFeature(){};

        Eigen::Vector3d line_length_;

        int map_id_ = -1;
        int pos_id_ = -1;
        int rot_id_ = -1;

        bool is_matched_ = false;
        bool is_fixed_ = false;

        int lost_ = -1;
        int obs_ = 0;

        std::vector<std::shared_ptr<CornerObservation>> cornersObs_;


        void GetValidMeasurements(const std::vector<double>& clone_times, std::vector<int>& valid_indexs)
        {

            for(int i = 0; i < cornersObs_.size(); i++)
            {
                double timestamp = cornersObs_[i]->timestamp_;
                if (std::find(clone_times.begin(), clone_times.end(), timestamp) != clone_times.end())
                {
                    valid_indexs.push_back(i);
                }
            }
        }
    };

    class DirectionFeature
    {
    public:
        Eigen::Vector3d direction_;
        std::vector<int> map_ids_;
    };


    /// plane parameters
    class PlaneSegmentCoeff
    {
    public:
        int id_ = -1;
        int plane_id_ = -1;
        Eigen::Vector3d normal_;
        double d_;
        Eigen::Vector3d center_;

        std::vector<Eigen::Vector3d> points_cloud_;
        std::vector<Eigen::Vector3d> convex_hull_;
        std::vector<cv::Point2f> measurements_;
        std::vector<cv::Point2f> meas_convex_;
        double MinDistanceFromSegment(const std::shared_ptr<PlaneSegmentCoeff> segment);
        bool ArePointsOnSameSide(const std::shared_ptr<PlaneSegmentCoeff> segment);
    };

    /// plane parameters
    class PlaneCoeff
    {
    public:
        Eigen::Vector3d normal_;
        double d_;

        std::vector<Eigen::Vector3d> points_cloud_;  // point cloud from depth
        std::vector<Eigen::Vector3d> convex_hull_;  // convex hull of points_cloud_ projected on plane

        double GetConvexHullArea();
        double MinDistanceFromSegment(const std::shared_ptr<PlaneSegmentCoeff> plane);
        double MinDistanceFromPlane(const std::shared_ptr<PlaneCoeff> plane);
        double MaxDistanceFromPlane(const std::shared_ptr<PlaneCoeff> plane);
        double MinDistanceAlongPlane(const std::shared_ptr<PlaneCoeff> plane);
        double ComputeParallelOverlap(const std::shared_ptr<PlaneCoeff> plane);
        bool ArePointsOnSameSide(const std::shared_ptr<PlaneCoeff> plane);
    };


    /// plane observation
    class PlaneObservation
    {
    public:
        PlaneObservation()
        {
            coeff_ = std::make_shared<PlaneCoeff>();
        };

        void SetPose(Eigen::Matrix<double,3,3> R_GtoI, Eigen::Matrix<double,3,1> p_IinG,
                     Eigen::Matrix<double,3,3> R_ItoC, Eigen::Matrix<double,3,1> p_IinC);
        void SetPose(Eigen::Matrix<double,3,3> R_GtoC, Eigen::Matrix<double,3,1> p_CinG);

        std::shared_ptr<PlaneCoeff> ConvertToWorldCoeff(std::shared_ptr<PlaneCoeff>& coeff);


        int id_ = -1;
        std::shared_ptr<PlaneCoeff> coeff_; // {C}
        std::vector<std::shared_ptr<PlaneSegmentCoeff>> segments_;  //


        bool to_delete_ = false;
        size_t feat_class_ = 0;
        double timestamp_;

        int map_id_ = -1;


        Eigen::MatrixXd R_ = Eigen::MatrixXd::Zero(3,3); // {C}

//        Eigen::Matrix<double,3,3> R_GtoIi_;
//        Eigen::Matrix<double,3,1> p_IiinG_;
//        Eigen::Matrix<double,3,3> R_ItoC_;
//        Eigen::Matrix<double,3,1> p_IinC_;

        Eigen::Matrix<double,3,3> R_GtoC_;
        Eigen::Matrix<double,3,1> p_CinG_;


        std::vector<int> border_ids_;
    };

    /// plane feature
    class PlaneFeature
    {
    public:
        PlaneFeature(){
            coeff_ = std::make_shared<PlaneCoeff>();
            corner_ids_.clear();
        };

        void GetValidMeasurements(const std::vector<double>& clone_times, std::vector<int>& valid_index);

//        size_t feat_class_ = 0;  //
        int id_ = -1;
        int map_id_ = -1;
        int lost_ = -1; //
        int obs_ = 0;  //
        bool is_matched_ = false;  //
        bool is_corner_ = false;  //
        double timestamp_;  //


        std::shared_ptr<PlaneCoeff> coeff_;
        std::vector<std::shared_ptr<PlaneObservation>> observations_;

        bool to_delete_ = false;  //
        bool can_relocate_ = false;  //

        int dir_id_ = -1;  //

        bool is_fixed_ = false; //
        bool is_marg = false;  //
        std::unordered_map<int, int> corner_ids_;  //

    };



}  // namespace

#endif // RGBD_PLANE_VIO_PLANE_FEATURE_H