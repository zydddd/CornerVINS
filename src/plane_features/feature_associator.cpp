//
// Created by zyd on 24-5-23.
//

#include "feature_associator.h"
using namespace RvgVio;

FeatureAssociator::FeatureAssociator(double angle, double dis, double overlap, bool use_HM, bool share_rotation)
{
    angle_thres_ = angle;
    dis_thres_ = dis;
    overlap_thres_ = overlap;

    use_hierarchical_matching_ = use_HM;
    share_rotation_ = share_rotation;

    planeFeatureId = 0;
    cornerFeatureId = 0;
    posFeatureId = 0;
    rotFeatureId = 0;
    directionFeatureId = 0;

    // x->x, y->y, z->z
    R_align.resize(3, std::vector<int>(3,0));
    R_align[0][0] = 0;
    R_align[0][1] = 1;
    R_align[0][2] = 2;
    // z->x, x->y, y->z
    R_align[1][0] = 1;
    R_align[1][1] = 2;
    R_align[1][2] = 0;
    // y->x, z->y, x->z
    R_align[2][0] = 2;
    R_align[2][1] = 0;
    R_align[2][2] = 1;

    M_align.resize(8, Eigen::Matrix3d::Zero());
    // x->x, y->y, z->z
    M_align[0] = Eigen::Matrix3d::Identity();
    // x->y, y->-x,z->z
    M_align[1](0,1) = 1;
    M_align[1](1,0) = -1;
    M_align[1](2,2) = 1;
    // x->-x, y->-y, z->z
    M_align[2](0,0) = -1;
    M_align[2](1,1) = -1;
    M_align[2](2,2) = 1;
    // x->-y, y->x, z->z
    M_align[3](0,1) = -1;
    M_align[3](1,0) = 1;
    M_align[3](2,2) = 1;
    // x->y, y->x, z->-z
    M_align[4](0,1) = 1;
    M_align[4](1,0) = 1;
    M_align[4](2,2) = -1;
    // x->x, y->-y, z->-z
    M_align[5](0,0) = 1;
    M_align[5](1,1) = -1;
    M_align[5](2,2) = -1;
    // x->-y, y->-x, z->-z
    M_align[6](0,1) = -1;
    M_align[6](1,0) = -1;
    M_align[6](2,2) = -1;
    // x->-x, y->y, z->-z
    M_align[6](0,0) = -1;
    M_align[6](1,1) = 1;
    M_align[6](2,2) = -1;
}


bool FeatureAssociator::IsMatch(const std::shared_ptr<PlaneCoeff>& p1, const std::shared_ptr<PlaneCoeff>& p2)
{
    double angle = acosf(std::min(1.0,abs(p1->normal_.transpose() * p2->normal_)));  // 0-pi/2

    std::cout << "[Is Match] angle = " << angle << " ";
    if(angle < angle_thres_ * M_PI / 180.0)
    {
        double d12 = p1->MinDistanceFromPlane(p2);
        std::cout << "distance = " << d12 << " ";
        if(d12 < dis_thres_)
        {
            double overlap = p1->ComputeParallelOverlap(p2);
            std::cout << "overlap = " << overlap << " ";

            if(overlap > overlap_thres_)
            {
//                debug_file << "IsMatch: " << p1->normal_.transpose() << ", " << p2->normal_.transpose() << ", " <<  p1->d_ << ", " << p2->d_ << std::endl;
//                debug_file << angle << " " << d12 << " " << overlap << std::endl << std::endl;
                return true;
            }

        }

    }
    std::cout << std::endl;
    return false;
}

double FeatureAssociator::MatchScore(const std::shared_ptr<PlaneCoeff>& p1, const std::shared_ptr<PlaneCoeff>& p2)
{
    double score = -1.0;
    double angle = acosf(std::min(1.0,abs(p1->normal_.transpose() * p2->normal_)));  // 0-pi/2
//    std::cout << "[Match Score] angle = " << angle;
    if(angle < angle_thres_ * M_PI / 180.0)
    {
        double d12 = p1->MinDistanceFromPlane(p2);
//        std::cout << ", distance = " << d12;
        if(d12 < dis_thres_)
        {
            double overlap = p1->ComputeParallelOverlap(p2);
//            std::cout << ", overlap = " << overlap;
            if(overlap > overlap_thres_)
            {
//                score = exp(-angle) + exp(-d12) + 1 - exp(overlap);
                double score_angle = 1 - 1 / (1 + exp(-8 * (angle - 0.5)));
                double score_dis = 1 - 1 / (1 + exp(-5 * (d12 - 1)));
                double score_overlap = 0;
                score = score_angle + score_dis + score_overlap;
            }
        }

    }
    std::cout << score << " ";
    return score;
}



void FeatureAssociator::PlaneMatching(std::shared_ptr<FeatureGraph> mapGraph,
                                      std::vector<std::shared_ptr<CornerObservation>>& corners,
                                 std::vector<std::shared_ptr<PlaneObservation>>& planes,
                                 Eigen::Matrix<double, 3, 3>& R_GtoCi,
                                 Eigen::Matrix<double, 3, 1>& p_CiinG)
{
    int nD = planes.size();
    int nM = mapGraph->n;

    if(nD == 0)
    {
        std::cout << "[Plane Matching] No plane is detected." << std::endl;
        return;
    }

    if(use_hierarchical_matching_)
    {
        for(int i = 0; i < corners.size(); i++)
        {
            if(!corners[i]->is_matched_) continue;

            int corner_map_id = corners[i]->map_id_;
            auto corner_landmark = mapGraph->cornersIdLookup_.at(corner_map_id);

            Eigen::Matrix3d R_M = corners[i]->R_MtoM0_;
            for(int k = 0; k < 3; k++)
            {
                int map_plane_id = corner_landmark->plane_ids_[k];
                if(map_plane_id == -1 || mapGraph->index_[map_plane_id] == -1) continue;

                for(int j = 0; j < 3; j++)
                {
                    if(R_M(j,k) == 1 || R_M(j,k) == -1)
                    {
                        int plane_id = corners[i]->plane_ids_[j];

                        planes[plane_id]->map_id_ = map_plane_id;
                        auto plane_landmark = mapGraph->GetPlane(map_plane_id);
                        plane_landmark->is_matched_ = true;
                        std::cout << "[Hierarchical Matching] " << plane_id << ", " << plane_landmark->id_ << "[" << map_plane_id << "]" << std::endl;
                        break;
                    }
                }
            }
        }
    }


    if(use_local_tracking_)
    {
        for(int i = 0; i < nD; i++)
        {
            if (planes[i]->map_id_ != -1) continue;

            auto plane_world_coeff = planes[i]->ConvertToWorldCoeff(planes[i]->coeff_);

            double score = -1.0;
            int index = -1;

            for (auto it = mapGraph->local_map_ids_.begin(); it != mapGraph->local_map_ids_.end(); it++) {
                int map_id = (*it);

                if(mapGraph->index_.find(map_id) == mapGraph->index_.end()) continue;

                int j = mapGraph->index_[map_id];
                if (mapGraph->vertices_[j]->is_matched_) continue;

                std::cout << "[Local Tracking] " << i << ", " << j << "[" << map_id << "]: ";
                auto landmark = mapGraph->vertices_[j];

                double score_test = MatchScore(plane_world_coeff, landmark->coeff_);
                if (score_test > 0 && score_test > score)
                {
                    score = score_test;
                    index = j;
                }
            }
            if(index != -1)
            {
                int map_id = mapGraph->vertices_[index]->map_id_;
                planes[i]->map_id_ = map_id;
                mapGraph->vertices_[index]->is_matched_ = true;
                std::cout << " [Local Tracking] " << i << ", " << index << "[" << map_id << "] success !!!" << std::endl;
            }
            else
            {
                std::cout << std::endl;
            }
        }
    }


    for (int i = 0; i < nD; i++)
    {
        if (planes[i]->map_id_ != -1) continue;


        auto plane_world_coeff = planes[i]->ConvertToWorldCoeff(planes[i]->coeff_);

        Eigen::Matrix<double,3,3> R_GtoCi = planes[i]->R_GtoC_;
        Eigen::Matrix<double,3,1> p_CioinG = planes[i]->p_CinG_;
//        Eigen::Matrix<double,3,3> R_GtoCi = obs[i]->R_ItoC_ * obs[i]->R_GtoIi_;
//        Eigen::Matrix<double,3,1> p_CioinG = obs[i]->p_IiinG_ - R_GtoCi.transpose() * obs[i]->p_IinC_;
        Eigen::Matrix<double,3,3> R_CitoG = R_GtoCi.transpose();
        Eigen::Matrix<double,3,1> p_GinCi = - R_GtoCi * p_CioinG;

        for(int j = 0; j < nM; j++)
        {
            auto plane_landmark = mapGraph->vertices_[j];

            if (plane_landmark->is_matched_)
                continue;

            std::cout << "[Force Matching] " << i << ", " << j << "[" << plane_landmark->map_id_ << "]: ";

            if (IsMatch(plane_world_coeff, plane_landmark->coeff_))
            {
                int map_id = plane_landmark->map_id_;
                planes[i]->map_id_ = map_id;
                plane_landmark->is_matched_ = true;
                std::cout << " [Force Tracking] " << i << ", " << j << "[" << map_id << "] success !!!" << std::endl;
                break;
            }
        }
        std::cout << std::endl;

        if (planes[i]->map_id_ == -1)
        {
            int map_id = planeFeatureId++;
            planes[i]->map_id_ = map_id;
        }
    }

    // debug
    {
        std::cout << "[Plane Matching] Assign map id: ";
        for (int i = 0; i < nD; i++)
        {
            std::cout << planes[i]->map_id_ << " ";
        }
        std::cout << std::endl << std::endl;
    }

    /// Update map
    mapGraph->AppendNewPlaneObservations(planes);

    PlaneCornerAssociate(mapGraph, corners, planes);
}


void FeatureAssociator::PlaneCornerAssociate(std::shared_ptr<FeatureGraph> mapGraph,
                          std::vector<std::shared_ptr<CornerObservation>>& corners,
                          std::vector<std::shared_ptr<PlaneObservation>>& planes)
{
    for(int i = 0; i < corners.size(); i++)
    {
        int corner_map_id = corners[i]->map_id_;
        auto corner_landmark = mapGraph->cornersIdLookup_.at(corner_map_id);

        if(corners[i]->is_matched_)
        {
            Eigen::Matrix3d R_M = corners[i]->R_MtoM0_;
            for(int k = 0; k < 3; k++)
            {
                int map_plane_id = corner_landmark->plane_ids_[k];
                if(map_plane_id == -1 || mapGraph->index_[map_plane_id] == -1)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (R_M(j, k) == 1 || R_M(j, k) == -1)
                        {
                            int plane_id = corners[i]->plane_ids_[j];
                            int new_plane_map_id = planes[plane_id]->map_id_;
                            auto plane_landmark = mapGraph->GetPlane(new_plane_map_id);

                            corner_landmark->plane_ids_[k] = new_plane_map_id;
                            if (plane_landmark->corner_ids_.find(corner_map_id) == plane_landmark->corner_ids_.end()) {
                                plane_landmark->corner_ids_[corner_map_id] = j;
                            }
                            break;
                        }
                    }
                }

                auto plane_landmark = mapGraph->GetPlane(corner_landmark->plane_ids_[k]);
                plane_landmark->is_corner_ = true;

            }
        }
        else
        {
            for(int j = 0; j < 3; j++)
            {
                int plane_id = corners[i]->plane_ids_[j];
                int plane_map_id = planes[plane_id]->map_id_;
                auto plane_landmark = mapGraph->GetPlane(plane_map_id);


                corner_landmark->plane_ids_[j] = plane_map_id;
                if(plane_landmark->corner_ids_.find(corner_map_id) == plane_landmark->corner_ids_.end())
                {
                    plane_landmark->corner_ids_[corner_map_id] = j;
                }

                plane_landmark->is_corner_ = true;
            }
        }
    }
}

int FeatureAssociator::DoubleCheck(std::shared_ptr<FeatureGraph> mapGraph,
                              Eigen::Vector3d& direction)
{
//    int dir_map_id = -1;
//    for (auto it = mapGraph->directionsIdLookup_.begin(); it != mapGraph->directionsIdLookup_.end(); it++)
//    {
//        double angle = acosf(std::min(1.0,abs(direction.transpose() * (*it).second->direction_)));  // 0-pi/2
//        if(direction.transpose() * (*it).second->direction_ >0.8)
////        if (angle < 10 * M_PI / 180)
//        {
//            dir_map_id = (*it).first;
//            break;
//        }
//    }
//    return dir_map_id;
}

void FeatureAssociator::HierarchicalMatching(std::shared_ptr<FeatureGraph> mapGraph,
                                        std::vector<std::shared_ptr<CornerObservation>>& corners,
                                        std::vector<std::shared_ptr<PlaneObservation>>& planes,
                                        Eigen::Matrix<double, 3, 3>& R_GtoCi, Eigen::Matrix<double, 3, 1>& p_CiinG)
{

    std::cout << "============= Corner Matching" << std::endl;
    CornerMatching(mapGraph, corners, planes, R_GtoCi, p_CiinG);

    std::cout << "============= Plane Matching" << std::endl;
    PlaneMatching(mapGraph, corners, planes, R_GtoCi, p_CiinG);

//    std::cout << "============= Direction Matching" << std::endl;
//    DirectionMatching(mapGraph, corners, planes, R_GtoCi, p_CiinG);

}


bool FeatureAssociator::IsRotationMatch(Eigen::Matrix<double, 3, 3>& R_MtoG,
                                        Eigen::Matrix<double, 3, 3>& R_M0toG,
                                       Eigen::Matrix<double, 3, 3>& R_MtoM0)
{
//    for(int i = 0; i < 8; i++)
//    {
//        Eigen::Matrix<double, 3, 3> R_MtoM1 = M_align[i];
    Eigen::Matrix<double, 3, 3> R_MtoM1 = Eigen::Matrix3d::Identity();

        for (int r = 0; r < 3; r++)
        {
            // xyz
            Eigen::Matrix<double, 3, 3> R_M1toM0 = Eigen::MatrixXd::Zero(3, 3);
            R_M1toM0(0, R_align[r][0]) = 1;
            R_M1toM0(1, R_align[r][1]) = 1;
            R_M1toM0(2, R_align[r][2]) = 1;

            R_MtoM0 = R_M1toM0 * R_MtoM1;
            Eigen::Matrix<double, 3, 3> rotation_error = R_MtoG.transpose() * R_M0toG * R_MtoM0;
            double trace = rotation_error.trace();
            double rad_error = acosf(std::max(-1.0,std::min(1.0,(trace - 1.0) / 2.0)));
            double angle_error = rad_error / M_PI * 180;

            std::cout << "angle error " << (trace - 1.0) / 2.0 << std::endl;
            if (angle_error < rotation_angle_error_)
            {
                return true;
            }
        }
//    }

    return false;
}
void FeatureAssociator::CornerMatching(std::shared_ptr<FeatureGraph> mapGraph,
                                  std::vector<std::shared_ptr<CornerObservation>>& corners,
                                  std::vector<std::shared_ptr<PlaneObservation>>& planes,
                                  Eigen::Matrix<double, 3, 3>& R_GtoCi, Eigen::Matrix<double, 3, 1>& p_CiinG)
{
    int nD = corners.size();
    if(nD == 0)
    {
        std::cout << "[Corner Matching] No corner is detected." << std::endl;
        return;
    }

    /// Associate corners
    for(int c = 0; c < corners.size(); c++)
    {

        std::shared_ptr<CornerObservation> corner_observation = corners[c];


        Eigen::Vector3d position_G = R_GtoCi.transpose() * corner_observation->position_ + p_CiinG;
        Eigen::Matrix<double, 3, 3> R_M0toG = R_GtoCi.transpose() * corner_observation->R_M0toC_;


        for (auto it = mapGraph->cornersIdLookup_.begin(); it != mapGraph->cornersIdLookup_.end(); it++)
        {

            int corner_map_id = (*it).first;
            std::shared_ptr<CornerFeature> corner_landmark = (*it).second;


            if (corner_landmark->is_matched_) continue;

            Eigen::Vector3d position = mapGraph->GetCornerPosition(corner_map_id);

            if ((position_G - position).norm() > position_error_)
                continue;

            Eigen::Matrix<double, 3, 3> R_MtoG = mapGraph->GetCornerRotation(corner_map_id);
            Eigen::Matrix<double, 3, 3> R_MtoM0;
            if(IsRotationMatch(R_MtoG, R_M0toG, R_MtoM0))
            {
                corner_observation->map_id_ = corner_landmark->map_id_;
                corner_observation->rot_map_id_ = corner_landmark->rot_id_;
                corner_observation->pos_map_id_ = corner_landmark->pos_id_;
                corner_observation->R_MtoM0_ = R_MtoM0;
                corner_observation->is_matched_ = true;
                corner_landmark->is_matched_ = true;
                break;
            }

            std::cout << "R " << R_MtoM0 << std::endl;
        }

        if (corner_observation->map_id_ == -1)  ///
        {
            int map_id = cornerFeatureId++;
            corner_observation->map_id_ = map_id;


            int pos_id = posFeatureId++;
            corner_observation->pos_map_id_ = pos_id;

            if(share_rotation_)
            {

                for (auto it = mapGraph->rotationsIdLookup_.begin(); it != mapGraph->rotationsIdLookup_.end(); it++)
                {
                    int rot_id = (*it).first;
                    Eigen::Matrix3d R_MtoG = (*it).second;
                    Eigen::Matrix<double, 3, 3> R_MtoM0;
                    if(IsRotationMatch(R_MtoG, R_M0toG, R_MtoM0))
                    {
                        corner_observation->rot_map_id_ = rot_id;
                        corner_observation->R_MtoM0_ = R_MtoM0;
                        break;
                    }
                }
            }

            if(corner_observation->rot_map_id_ == -1)
            {
                int rot_id = rotFeatureId++;
                corner_observation->rot_map_id_ = rot_id;
                corner_observation->R_MtoM0_ = Eigen::MatrixXd::Identity(3, 3);
            }
        }
    }

    ////// debug
    std::cout << "[Corner Matching] Assign map id: ";
    for (int i = 0; i < nD; i++)
    {
        std::cout << corners[i]->map_id_ << ": (" << corners[i]->rot_map_id_ << ", " << corners[i]->pos_map_id_ << ") ";
    }
    std::cout << std::endl << std::endl;

    /// Update corner map
    mapGraph->AppendNewCornerObservations(corners);
}
