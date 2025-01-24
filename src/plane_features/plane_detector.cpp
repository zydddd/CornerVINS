//
// Created by zyd on 23-12-12.
//

#include <iostream>
#include <thread>
#include <mutex>
#include <opencv2/core/core.hpp>

#include "plane_feature.h"
#include "plane_detector.h"

using namespace Rvg;

PlaneDetector::PlaneDetector(Eigen::VectorXd camera_calib, double kScaleFactor, double angle, double dis, double n)
{
    double fx = camera_calib(0);
    double fy = camera_calib(1);
    double cx = camera_calib(2);
    double cy = camera_calib(3);

    cv::Mat tempK =
            (cv::Mat_<double>(3, 3) << fx, 0, cx,
                    0, fy, cy,
                    0, 0, 1);
    K_ = tempK;
    kScaleFactor_ = 1 / kScaleFactor;

    angle_thres = angle;
    dis_thres = dis;
    noise = n;
}

PlaneDetector::~PlaneDetector()
{
    cloud.vertices.clear();
    seg_img_.release();
    color_img_.release();
}

bool PlaneDetector::ReadColorImage(cv::Mat& RGBImg)
{
    color_img_ = RGBImg.clone();
    if (color_img_.empty() || color_img_.depth() != CV_8U)
    {
        std::cout << "ERROR: cannot read color image. No such a file, or the image format is not 8UC3." << std::endl;
        return false;
    }
    return true;
}

bool PlaneDetector::ReadDepthImage(cv::Mat& depthImg)
{
    cv::Mat depth_img = depthImg.clone();
    if (depth_img.empty() || depth_img.depth() != CV_16U)
    {
        std::cout << "WARNING: cannot read depth image. No such a file, or the image format is not 16UC1" << std::endl;
        return false;
    }
    depth_img_ = depthImg.clone();
    return true;
}


void PlaneDetector::AHCPlaneSegmentation()
{
    cloud.vertices.clear();
    auto cols = depth_img_.cols;
    auto rows = depth_img_.rows;
    cloud.vertices.resize(rows * cols);
    cloud.w = cols;
    cloud.h = rows;

    int vertex_idx = 0;
    for (int i = 0; i < rows; i+=1)
    {
        for (int j = 0; j < cols; j+=1)
        {
            double z = (double)(depth_img_.at<unsigned short>(i, j)) * kScaleFactor_;
            if (isnan(z) || z < valid_depth_near_ || z > valid_depth_far_)  //
            {
                cloud.vertices[vertex_idx++] = Eigen::Vector3d(0, 0, z);
                continue;
            }
            double x = ((double)j - K_.at<double>(0, 2)) * z / K_.at<double>(0, 0);
            double y = ((double)i - K_.at<double>(1, 2)) * z / K_.at<double>(1, 1);
            cloud.vertices[vertex_idx++] = Eigen::Vector3d(x, y, z);
        }
    }

    plane_vertices_.clear();
    int kDepthWidth = cols;
    int kDepthHeight = rows;

//    seg_img_ = cv::Mat(kDepthHeight, kDepthWidth, CV_8UC1);
    seg_img_ = cv::Mat(kDepthHeight, kDepthWidth, CV_8UC3);

    std::chrono::steady_clock::time_point t_before = std::chrono::steady_clock::now();
    plane_filter = ahc::PlaneFitter<ImagePointCloud>();
    plane_filter.run(&cloud, &plane_vertices_, &seg_img_);
    std::chrono::steady_clock::time_point t_end = std::chrono::steady_clock::now();
    double wait_detect = std::chrono::duration_cast<std::chrono::duration<double> >(t_end - t_before).count() * 1000;

    std::cout << "[TIME] AHC plane detection: " << wait_detect << " ms." << std::endl;
    std::cout << "[AHC] detected plane num: " << (int)plane_vertices_.size() << std::endl;

}

void PlaneDetector::AppendNewPlanes(double timestamp)
{
    n_sp.clear();
    n_ds.clear();
    n_fit.clear();
    n_ds_merge.clear();
    n_refit.clear();

    plane_detected.clear();
    int num = 0;

    for (int i = 0; i < (int)plane_vertices_.size(); i++)
    {

        // ax+by+cz+d=0
        auto extractedPlane = plane_filter.extractedPlanes[i];
        double nx = extractedPlane->normal[0];
        double ny = extractedPlane->normal[1];
        double nz = extractedPlane->normal[2];
        double cx = extractedPlane->center[0];
        double cy = extractedPlane->center[1];
        double cz = extractedPlane->center[2];
        double d = (double) -(nx * cx + ny * cy + nz * cz);
        cv::Mat coef = (cv::Mat_<double>(4, 1) << nx, ny, nz, d);

        /// 过滤外点并重估计平面参数
        auto &indices = plane_vertices_[i];
        std::vector<bool> valid(depth_img_.cols * depth_img_.rows,true);


        std::vector<std::vector<int>> indexs_col(depth_img_.cols, std::vector<int>());
        std::vector<cv::Point2f> meas;  //
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>());


        if(refine_segmentation_)
        {
            std::vector<std::vector<int>> indexs_col(depth_img_.cols, std::vector<int>());
            std::vector<std::vector<int>> indexs_row(depth_img_.rows, std::vector<int>());
            for (int j : indices)
            {
                ///
                indexs_col[j % depth_img_.cols].push_back(j);
                ///
                indexs_row[j / depth_img_.cols].push_back(j);
            }

            //
            int max_height = 0;
            for(int col = 0; col < depth_img_.cols; col++)
            {
                if(indexs_col[col].size() > max_height)
                    max_height = indexs_col[col].size();
            }

            for(int col = 0; col < depth_img_.cols; col++)
            {
                if(indexs_col[col].size() < 0.3 * max_height)
                {
                    continue;
                    for(int p = 0; p < indexs_col[col].size(); p++)
                    {
                        int j = indexs_col[col][p];
                        valid[j] = false;
                    }
                }

                {
                    for (int p = 0; p < indexs_col[col].size(); p++) {
                        int j = indexs_col[col][p];
                        pcl::PointXYZRGB pt;
                        pt.x = (float) cloud.vertices[j][0];
                        pt.y = (float) cloud.vertices[j][1];
                        pt.z = (float) cloud.vertices[j][2];
                        if (pt.z <= valid_depth_near_ || pt.z > valid_depth_far_)
                            continue;
                        inputCloud->points.push_back(pt);

                        float x = pt.x / pt.z * K_.at<double>(0, 0) + K_.at<double>(0, 2);
                        float y = pt.y / pt.z * K_.at<double>(1, 1) + K_.at<double>(1, 2);
                        meas.push_back(cv::Point2f(x, y));
                    }
                }
            }

//            //
//            int max_width = 0;
//            for(int row = 0; row < depth_img_.rows; row++)
//            {
//                if(indexs_row[row].size() > max_width)
//                    max_width = indexs_row[row].size();
//            }
//
//            for(int row = 0; row < depth_img_.rows; row++)
//            {
//                if(indexs_row[row].size() < 0.3 * max_width)
//                {
//
//                    for(int p = 0; p < indexs_row[row].size(); p++)
//                    {
//                        int j = indexs_row[row][p];
//                        valid[j] = false;
//                    }
//                }
//            }
        }

        else
        {
            for (int j : indices)
            {
//                if(!valid[j]) continue;

                pcl::PointXYZRGB pt;
                pt.x = (float) cloud.vertices[j][0];
                pt.y = (float) cloud.vertices[j][1];
                pt.z = (float) cloud.vertices[j][2];
                if(pt.z <= valid_depth_near_ || pt.z > valid_depth_far_)
                    continue;
                inputCloud->points.push_back(pt);

                float x = pt.x / pt.z * K_.at<double>(0, 0) + K_.at<double>(0, 2);
                float y = pt.y / pt.z * K_.at<double>(1, 1) + K_.at<double>(1, 2);
                meas.push_back(cv::Point2f(x, y));
            }
        }


        n_sp.push_back(inputCloud->points.size());

        if(inputCloud->points.size() < size_thres)
            continue;

//        pcl::visualization::CloudViewer viewer("Cloud Viewer");
//        viewer.showCloud(inputCloud);
//        while (!viewer.wasStopped());
//        inputCloud->points.clear();



        pcl::VoxelGrid<pcl::PointXYZRGB> voxel;
        voxel.setLeafSize(downsample_leaf_size, downsample_leaf_size, downsample_leaf_size);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
        voxel.setInputCloud(inputCloud);
        voxel.filter(*cloud_temp);
//        coarseCloud = inputCloud;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr coarseCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        coarseCloud = cloud_temp;

        std::shared_ptr<PlaneSegmentCoeff> segment = std::make_shared<PlaneSegmentCoeff>();

        if(refine_ahc_detect)
        {
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //
            bool valid = MaxPointDistanceFromPlane(coef, coarseCloud, inliers);  //

            n_ds.push_back(coarseCloud->points.size());
            n_fit.push_back(inliers->indices.size());

            if (!valid)
            {
                continue;
            }

            // 新参数
            nx = coef.at<double>(0);
            ny = coef.at<double>(1);
            nz = coef.at<double>(2);
            d = coef.at<double>(3);

            std::cout << nx << " " << ny << " " << nz << " " << d << std::endl;

            std::vector<cv::Point2f> inlier_means;  //
            Eigen::Vector3d center(0,0,0);
            for (int ind: inliers->indices)
            {
                auto point = coarseCloud->points[ind];
                Eigen::Vector3d p;
                p(0) = (double) point.x;
                p(1) = (double) point.y;
                p(2) = (double) point.z;
                center = center + p;
                segment->points_cloud_.push_back(p);

                float x = point.x / point.z * K_.at<double>(0, 0) + K_.at<double>(0, 2);
                float y = point.y / point.z * K_.at<double>(1, 1) + K_.at<double>(1, 2);
                inlier_means.push_back(cv::Point2f(x, y));

            }
            center = center / inliers->indices.size();
            segment->center_ = center;
            segment->measurements_ = meas;
        }
        else
        {
            Eigen::Vector3d center(0,0,0);
            for (auto point: coarseCloud->points)
            {
                Eigen::Vector3d p;
                p(0) = (double) point.x;
                p(1) = (double) point.y;
                p(2) = (double) point.z;
                center = center + p;
                segment->points_cloud_.push_back(p);
            }
            center = center / coarseCloud->points.size();
            segment->center_ = center;
            segment->measurements_ = meas;
        }


        //
        std::vector<cv::Point2f> convexHullPoints2D;
        cv::convexHull(segment->measurements_, convexHullPoints2D, true, true);
        segment->meas_convex_ = convexHullPoints2D;

        //
        double norm = sqrt(nx * nx + ny * ny + nz * nz);
        nx = nx / norm;
        ny = ny / norm;
        nz = nz / norm;
        d = d / norm;

        Eigen::MatrixXd planeCP = Eigen::MatrixXd::Zero(3,1);
        planeCP << nx, ny, nz;
        planeCP = planeCP / planeCP.norm() * (-d);

        /// valid plane segment
        segment->normal_ = planeCP / planeCP.norm();
        segment->d_ = planeCP.norm();

        std::vector<Eigen::Vector3d> pointsProjected3D = GetPointsProjected(segment->points_cloud_, segment->normal_, segment->d_);
        std::vector<Eigen::Vector3d> pointsConvexHull = GetPointsConvexHull(pointsProjected3D, segment->normal_);
        segment->convex_hull_ = pointsConvexHull;

        if(merge_segments)
        {
            int match_to = -1;
            for(int k = 0; k < plane_detected.size(); k++)
            {
                std::shared_ptr<PlaneObservation> plane = plane_detected[k];
                for(int s = 0; s < plane->segments_.size(); s++)  //
                {
                    if(IsMatch(segment, plane->segments_[s]))
                    {
                        match_to = k;
                        break;
                    }
                }
            }

            if(match_to != -1)  //
            {
                std::shared_ptr<PlaneObservation> plane_obs = plane_detected[match_to];
                plane_obs->segments_.push_back(segment);
            }
            else //
            {
                std::shared_ptr<PlaneObservation> plane_obs = std::make_shared<PlaneObservation>();
//                plane_obs->id_ = num++;
                //
//                plane_obs->coeff_->normal_ << segment->normal_(0), segment->normal_(1), segment->normal_(2);
//                plane_obs->coeff_->d_ = segment->d_;
//                plane_obs->coeff_->points_cloud_ = segment->points_cloud_;
//                plane_obs->coeff_->convex_hull_ = segment->convex_hull_;
                plane_obs->segments_.push_back(segment);
                plane_detected.push_back(plane_obs);
            }
        }
        else  //
        {
            std::shared_ptr<PlaneObservation> plane_obs = std::make_shared<PlaneObservation>();
            plane_obs->id_ = num++;
            plane_obs->coeff_->normal_ << segment->normal_(0), segment->normal_(1), segment->normal_(2);
            plane_obs->coeff_->d_ = segment->d_;
            plane_obs->coeff_->points_cloud_ = segment->points_cloud_;
            plane_obs->coeff_->convex_hull_ = segment->convex_hull_;
            plane_obs->timestamp_ = timestamp;
            plane_obs->segments_.push_back(segment);
            plane_detected.push_back(plane_obs);

        }

    }


    if(merge_segments)  //
    {
        auto it = plane_detected.begin();
        while(it != plane_detected.end())
        {
            auto plane = (*it);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            for (int s = 0; s < plane->segments_.size(); s++) {
                auto segment = plane->segments_[s];
                for (int k = 0; k < segment->points_cloud_.size(); k++)
                {
                    pcl::PointXYZRGB pt;
                    pt.x = (float) segment->points_cloud_[k][0];
                    pt.y = (float) segment->points_cloud_[k][1];
                    pt.z = (float) segment->points_cloud_[k][2];
                    inputCloud->points.push_back(pt);
                }
            }

            cv::Mat coef;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //
            bool valid = MaxPointDistanceFromPlane(coef, inputCloud, inliers);  //

            n_ds_merge.push_back(inputCloud->points.size());
            n_refit.push_back(inliers->indices.size());

            if (!valid) {
                it = plane_detected.erase(it);
                continue;
            }

            //
            double nx = coef.at<double>(0);
            double ny = coef.at<double>(1);
            double nz = coef.at<double>(2);
            double d = coef.at<double>(3);

            //
            double norm = sqrt(nx * nx + ny * ny + nz * nz);
            nx = nx / norm;
            ny = ny / norm;
            nz = nz / norm;
            d = d / norm;

            Eigen::MatrixXd planeCP = Eigen::MatrixXd::Zero(3, 1);
            planeCP << nx, ny, nz;
            planeCP = planeCP / planeCP.norm() * (-d);

            /// valid plane segment
            plane->coeff_->normal_ = planeCP / planeCP.norm();
            plane->coeff_->d_ = planeCP.norm();

            // point clouds
            plane->coeff_->points_cloud_.clear();
            for (int ind: inliers->indices) {
                auto point = inputCloud->points[ind];
                Eigen::Vector3d p;
                p(0) = (double) point.x;
                p(1) = (double) point.y;
                p(2) = (double) point.z;
                plane->coeff_->points_cloud_.push_back(p);
            }

            // convex hull points
            {
                std::vector<Eigen::Vector3d> pointsProjected3D = GetPointsProjected(plane->coeff_->points_cloud_, plane->coeff_->normal_, plane->coeff_->d_);
                std::vector<Eigen::Vector3d> pointsConvexHull = GetPointsConvexHull(pointsProjected3D, plane->coeff_->normal_);
                plane->coeff_->convex_hull_ = pointsConvexHull;
            }

            plane->id_ = num++;
            plane->timestamp_ = timestamp;
            it++;
        }
    }

    std::cout << "Refined plane num: " << plane_detected.size() <<std::endl;

    ComputeNoise();
}

void PlaneDetector::ComputeNoise()
{
    auto it = plane_detected.begin();
    while(it != plane_detected.end())
    {
        auto plane = (*it);

        /// Compute Noise Matrix
        Eigen::MatrixXd Rplane = Eigen::MatrixXd::Zero(3,3);
        if(noise < 1e-6)  // noise = 0
        {
            // Use inliers to model
            for (auto point: plane->coeff_->points_cloud_)
            {
                Eigen::MatrixXd pfi = Eigen::MatrixXd::Zero(3,1);
                pfi(0) = (double) point.x();
                pfi(1) = (double) point.y();
                pfi(2) = (double) point.z();

                double u = pfi(0) / pfi(2);
                double v = pfi(1) / pfi(2);
                double z = pfi(2);

                Eigen::MatrixXd Hdi = 1 / plane->coeff_->d_ * pfi.transpose() *
                                      (Eigen::MatrixXd::Identity(3,3) - plane->coeff_->normal_  * plane->coeff_->normal_ .transpose()) - plane->coeff_->normal_.transpose();
                Eigen::MatrixXd Hni = plane->coeff_->normal_ .transpose();
                //
                Eigen::MatrixXd Hfni = Eigen::MatrixXd::Zero(3,3);
                Hfni << z, 0, u, 0, z, v, 0, 0, 1;
                Eigen::MatrixXd Rpf = Eigen::MatrixXd::Zero(3,3);
                double sigma_x = 1 / K_.at<double>(0, 0);
                double sigma_y = 1 / K_.at<double>(1, 1);
                double sigma_z = std::max(0.04 * z, 0.01);
                Rpf << sigma_x * sigma_x, 0, 0, 0, sigma_y * sigma_y, 0, 0, 0, sigma_z * sigma_z;
                Eigen::MatrixXd Rfi = Hfni  * Rpf * Hfni.transpose();
                //
                Rplane = Rplane + Hdi.transpose() * (Hni * Rfi * Hni.transpose()).inverse() * Hdi;
            }
            Rplane = Rplane / plane->coeff_->points_cloud_.size();  //
            plane->R_ = Rplane.inverse();
        }
        else  //
        {
            plane->R_  << noise, 0, 0, 0, noise, 0, 0, 0, noise;
        }

        it++;
    }
}

bool PlaneDetector::MaxPointDistanceFromPlane(cv::Mat &plane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, pcl::PointIndices::Ptr inliers)
{


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);


    cloud_filtered = pointCloud;
    int n = cloud_filtered->points.size();

//    if(n <= 10) return false;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);  //
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dis_thres);
//    seg.setEpsAngle(angleTh_ * (M_PI / 180.0f));

    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);


    if (inliers->indices.size() < inlier_rate_ * n)
    {
        std::cout << "Could not estimate a planar model for the given initial plane." << std::endl;
        return false;
    }

    plane = cv::Mat_<double>(4, 1);
    plane.at<double>(0) = coefficients->values[0];
    plane.at<double>(1) = coefficients->values[1];
    plane.at<double>(2) = coefficients->values[2];
    plane.at<double>(3) = coefficients->values[3];


    return true;
}


bool PlaneDetector::IsMatch(const std::shared_ptr<PlaneSegmentCoeff>& p1, const std::shared_ptr<PlaneSegmentCoeff>& p2)
{
    double angle = acosf(std::min(1.0,abs(p1->normal_.transpose() * p2->normal_)));
    double distance = p2->MinDistanceFromSegment(p1);

    if((angle < angle_thres * M_PI / 180.0) && (distance < dis_thres))
        return true;
    else
        return false;
}

void PlaneDetector::RunBorderDetection(double timestamp)
{
    int num_plane = plane_detected.size();
    int num_seg = 0;
    std::vector<std::shared_ptr<PlaneSegmentCoeff>> segments;
    for(int i = 0; i < num_plane; i++)
    {
        for(int j = 0; j < plane_detected[i]->segments_.size(); j++)
        {
            plane_detected[i]->segments_[j]->id_ = num_seg++;
            plane_detected[i]->segments_[j]->plane_id_ = i;
            segments.push_back(plane_detected[i]->segments_[j]);
        }
    }


    border_detected.clear();
    std::vector<std::vector<int>> sMatrix(num_seg, std::vector<int>(num_seg, -1));  //

    int num = 0;

    /// save pair angles
    for(int i = 0; i < num_plane; i++)
    {
        auto p1 = plane_detected[i]->coeff_;

        for(int j = i + 1; j < num_plane; j++)
        {
            auto p2 = plane_detected[j]->coeff_;

            std::cout << i << ", " << j << ": ";

            ///
            for(int s1 = 0; s1 < plane_detected[i]->segments_.size(); s1++)
            {
                for(int s2 = 0; s2 < plane_detected[j]->segments_.size(); s2++)
                {

                    int sid1 = plane_detected[i]->segments_[s1]->id_;
                    int sid2 = plane_detected[j]->segments_[s2]->id_;

                    std::pair<Eigen::Vector3d, Eigen::Vector3d> endPoints;
                    if(ComputeIntersectionLine(endPoints, plane_detected[i]->segments_[s1], plane_detected[j]->segments_[s2]))
                    {
                        std::shared_ptr<BorderObservation> border = std::make_shared<BorderObservation>(i, j, endPoints);
                        border_detected.push_back(border);
                        sMatrix[sid1][sid2] = num;
                        sMatrix[sid2][sid1] = num;
                        num++;
                    }
                }
            }
        }
    }
    std::cout << "Border detected: " << num << std::endl;

    corner_detected.clear();
    for(int i = 0; i < num_seg; i++)
    {
        for(int j = i + 1; j < num_seg; j++)
        {
            if(sMatrix[i][j] == -1)  //
                continue;

            for(int k = j + 1; k < num_seg; k++)
            {
                if(sMatrix[i][k] == -1 || sMatrix[j][k] == -1)
                    continue;

                // plane k
                int plane_k = segments[k]->plane_id_;
                auto bk = border_detected[sMatrix[i][j]];
                Eigen::Vector3d  pk = bk->end_points_.first;
                Eigen::Vector3d  qk = bk->end_points_.second;
                Eigen::Vector3d  vk = pk - qk;
                double lk = vk.norm();
                vk = vk / lk;

                // plane j
                int plane_j = segments[j]->plane_id_;
                auto bj = border_detected[sMatrix[i][k]];
                Eigen::Vector3d  pj = bj->end_points_.first;
                Eigen::Vector3d  qj = bj->end_points_.second;
                Eigen::Vector3d  vj = pj - qj;
                double lj = vj.norm();
                vj = vj / lj;

                // plane i
                int plane_i = segments[i]->plane_id_;
                auto bi = border_detected[sMatrix[j][k]];
                Eigen::Vector3d  pi = bi->end_points_.first;
                Eigen::Vector3d  qi = bi->end_points_.second;
                Eigen::Vector3d  vi = pi - qi;
                double li = vi.norm();
                vi = vi / li;

                //
                double angle1 = acosf(std::min(1.0,abs(vk.transpose() * vj)));  // 0-pi/2
                double angle2 = acosf(std::min(1.0,abs(vk.transpose() * vi)));  // 0-pi/2
                double angle3 = acosf(std::min(1.0,abs(vj.transpose() * vi)));  // 0-pi/2
                if(angle1 < vertical_rad_thres_ || angle2 < vertical_rad_thres_ || angle3 < vertical_rad_thres_)
                    continue;


                Eigen::Vector3d intersection = computeBordersIntersection(pj, vj, pi, vi);

                //
                {
                    if ((qk - intersection).norm() > (pk - intersection).norm()) {
                        bk->end_points_.second = pk;
                        bk->end_points_.first = qk;
                        pk = bk->end_points_.first;
                        qk = bk->end_points_.second;
                        vk = pk - qk;
                        vk = vk / vk.norm();
                    }

                    if ((qj - intersection).norm() > (pj - intersection).norm()) {
                        bj->end_points_.second = pj;
                        bj->end_points_.first = qj;
                        pj = bj->end_points_.first;
                        qj = bj->end_points_.second;
                        vj = pj - qj;
                        vj = vj / vj.norm();

                    }

                    if ((qi - intersection).norm() > (pi - intersection).norm()) {
                        bi->end_points_.second = pi;
                        bi->end_points_.first = qi;
                        pi = bi->end_points_.first;
                        qi = bi->end_points_.second;
                        vi = pi - qi;
                        vi = vi / vi.norm();
                    }
                }

                //
                if((intersection - qk).norm() < endpoints_intersection_dis_ &&
                   (intersection - qj).norm() < endpoints_intersection_dis_ &&
                   (intersection - qi).norm() < endpoints_intersection_dis_)
                {
                    auto planek = plane_detected[plane_k]->coeff_;
                    auto planej = plane_detected[plane_j]->coeff_;
                    auto planei = plane_detected[plane_i]->coeff_;


                    ///
                    Eigen::Matrix<double, 3, 3> bases;
                    SolveMainDirections(vk, vj, vi, bases);  //

                    ///
                    if(refine_plane_params_)
                    {
                        planek->normal_ = bases.col(0);
                        planej->normal_ = bases.col(1);
                        planei->normal_ = bases.col(2);
                        // 这里不加 -
                        planek->d_ =  planek->normal_.transpose() * intersection;
                        planej->d_ =  planej->normal_.transpose() * intersection;
                        planei->d_ =  planei->normal_.transpose() * intersection;
                    }

                    ///
                    Eigen::Vector3d intersection_new = computePlanesIntersection(planek->normal_, planek->d_,
                                                                                 planej->normal_,  planej->d_,
                                                                                 planei->normal_,  planei->d_);

                    //
                    Eigen::Matrix<double,3,1> z_axis = QuatUtils::SkewX(vk) * vj;
                    if(z_axis.transpose() * vi < 0)  //
                    {
                        SolveMainDirections(vj, vk, vi, bases);
                        std::shared_ptr<CornerObservation> corner = std::make_shared<CornerObservation>(timestamp, plane_j, plane_k, plane_i, intersection_new, bases);
                        corner->pos_R_ = (plane_detected[plane_i]->R_ + plane_detected[plane_j]->R_ + plane_detected[plane_k]->R_) / 3;
                        corner->position_before = intersection;
                        corner->bases_before << vj, vk, vi;
                        corner->line_length_ << lj + (qj - intersection_new).norm(), lk + (qk - intersection_new).norm(), li + (qi - intersection_new).norm();
                        corner_detected.push_back(corner);
                    }
                    else
                    {
                        std::shared_ptr<CornerObservation> corner = std::make_shared<CornerObservation>(timestamp, plane_k, plane_j, plane_i, intersection_new, bases);
                        corner->pos_R_ = plane_detected[plane_i]->R_ + plane_detected[plane_j]->R_ + plane_detected[plane_k]->R_;
                        corner->position_before = intersection;
                        corner->bases_before << vk, vj, vi;
                        corner->line_length_ << lk + (qk - intersection_new).norm(), lj + (qj - intersection_new).norm(), li + (qi - intersection_new).norm();
                        corner_detected.push_back(corner);
                    }
                }
            }
        }
    }

    std::cout << "Corner detected: " << corner_detected.size() << std::endl;
}



bool PlaneDetector::ComputeIntersectionLine(std::pair<Eigen::Vector3d, Eigen::Vector3d>& endPoints,
                                            const std::shared_ptr<PlaneCoeff>& p1, const std::shared_ptr<PlaneCoeff>& p2)
{
    ///
    double angle = acosf(std::min(1.0,abs(p1->normal_.transpose() * p2->normal_)));
    if(angle < 15 * M_PI / 180.0)  //
    {
        std::cout << "Parallel planes, no intersection line." << std::endl;
        return false;
    }

    ///
    std::vector<Eigen::Vector3d> cv1 = p1->convex_hull_;
    std::vector<Eigen::Vector3d> cv2 = p2->convex_hull_;
    double d12, d21;

    if(p1->ArePointsOnSameSide(p2))  //
    {
        d12 = p1->MinDistanceFromPlane(p2);  //
    }
    else
    {
        d12 = 0;
    }

    if(p2->ArePointsOnSameSide(p1))
    {
        d21 = p2->MinDistanceFromPlane(p1);
    }
    else
    {
        d21 = 0;
    }

    if(d12 > intersection_d_thres_ || d21 > intersection_d_thres_)
    {
        std::cout << "The two planes are separated!!" << std::endl;
        return false;
    }

    ///
    // Compute the direction of the intersection line as the cross product of the normals
    Eigen::Vector3d direction = p1->normal_.cross(p2->normal_).normalized();
    // z=0
    Eigen::Vector3d pointOnLine((p2->normal_(1) * p1->d_ - p1->normal_(1) * p2->d_) / (p1->normal_(0) * p2->normal_(1) - p2->normal_(0) * p1->normal_(1)),
                                (p2->normal_(0) * p1->d_ - p1->normal_(0) * p2->d_) / (p2->normal_(0) * p1->normal_(1) - p1->normal_(0) * p2->normal_(1)),
                                0);

    // ：point = pointOnLine + t * direction

    std::pair<Eigen::Vector3d, Eigen::Vector3d> intersectionPoints1, intersectionPoints2;  // 分别表示在p1和p2上的交线，最终交线段取其交集
    if(!ComputeEndPoints(cv1, direction, pointOnLine, intersectionPoints1, intersection_d_thres_) ||
       !ComputeEndPoints(cv2, direction, pointOnLine, intersectionPoints2, intersection_d_thres_))
    {
        return false;
    }

    ///
    Eigen::Vector3d p11 = intersectionPoints1.first;
    Eigen::Vector3d p12 = intersectionPoints1.second;
    Eigen::Vector3d p21 = intersectionPoints2.first;
    Eigen::Vector3d p22 = intersectionPoints2.second;

    if((p11 - p12).transpose() * (p21 - p22) < 0) //
    {
        auto tmp = p21;
        p21 = p22;
        p22 = tmp;
    }

    ///
    if((p11 - p12).transpose() * (p12 - p21) > 0 || (p11 - p12).transpose() * (p22 - p11) > 0 )  //
    {
        std::cout << "The two edges have no intersection." << std::endl;
        return false;
    }
    else if((p11 - p12).transpose() * (p11 - p21) > 0 )
    {
        if((p11 - p12).transpose() * (p22 - p12) > 0)  //
        {
//            std::cout << "Edge 22." << std::endl;
            endPoints = std::make_pair(p21, p22);
        }
        else if((p11 - p12).transpose() * (p12 - p22) > 0)
        {
//            std::cout << "Edge 21." << std::endl;
            endPoints = std::make_pair(p21, p12);
        }
    }
    else if((p11 - p12).transpose() * (p21 - p11) > 0)
    {
        if((p11 - p12).transpose() * (p12 - p22) > 0)  //
        {
//            std::cout << "Edge 11." << std::endl;
            endPoints = std::make_pair(p11, p12);
        }
        else if((p11 - p12).transpose() * (p22 - p12) > 0)
        {
//            std::cout << "Edge 12." << std::endl;
            endPoints = std::make_pair(p11, p22);
        }
    }

    if((endPoints.first - endPoints.second).norm() < edge_legnth_) return false;  //
    return true;
}

bool PlaneDetector::ComputeIntersectionLine(std::pair<Eigen::Vector3d, Eigen::Vector3d>& endPoints,
                                            const std::shared_ptr<PlaneSegmentCoeff>& p1, const std::shared_ptr<PlaneSegmentCoeff>& p2)
{
    ///
    double angle = acosf(std::min(1.0,abs(p1->normal_.transpose() * p2->normal_)));
    if(angle < 15 * M_PI / 180.0)  //
    {
        std::cout << "Parallel planes, no intersection line." << std::endl;
        return false;
    }

    ///
    std::vector<Eigen::Vector3d> cv1 = p1->convex_hull_;
    std::vector<Eigen::Vector3d> cv2 = p2->convex_hull_;
    double d12, d21;

    if(p1->ArePointsOnSameSide(p2))  //
    {
        d12 = p1->MinDistanceFromSegment(p2);  //
    }
    else
    {
        d12 = 0;
    }

    if(p2->ArePointsOnSameSide(p1))
    {
        d21 = p2->MinDistanceFromSegment(p1);
    }
    else
    {
        d21 = 0;
    }

    if(d12 > intersection_d_thres_ || d21 > intersection_d_thres_)
    {
        std::cout << "The two planes are separated!!" << std::endl;
        return false;
    }

    ///
    // Compute the direction of the intersection line as the cross product of the normals
    Eigen::Vector3d direction = p1->normal_.cross(p2->normal_).normalized();
    //
    Eigen::Vector3d pointOnLine((p2->normal_(1) * p1->d_ - p1->normal_(1) * p2->d_) / (p1->normal_(0) * p2->normal_(1) - p2->normal_(0) * p1->normal_(1)),
                                (p2->normal_(0) * p1->d_ - p1->normal_(0) * p2->d_) / (p2->normal_(0) * p1->normal_(1) - p1->normal_(0) * p2->normal_(1)),
                                0);

    //：point = pointOnLine + t * direction

    std::pair<Eigen::Vector3d, Eigen::Vector3d> intersectionPoints1, intersectionPoints2;  //
    if(!ComputeEndPoints(cv1, direction, pointOnLine, intersectionPoints1, intersection_d_thres_) ||
       !ComputeEndPoints(cv2, direction, pointOnLine, intersectionPoints2, intersection_d_thres_))
    {
        return false;
    }

    ///
    Eigen::Vector3d p11 = intersectionPoints1.first;
    Eigen::Vector3d p12 = intersectionPoints1.second;
    Eigen::Vector3d p21 = intersectionPoints2.first;
    Eigen::Vector3d p22 = intersectionPoints2.second;

    if((p11 - p12).transpose() * (p21 - p22) < 0) //
    {
        auto tmp = p21;
        p21 = p22;
        p22 = tmp;
    }

    ///
    if((p11 - p12).transpose() * (p12 - p21) > 0 || (p11 - p12).transpose() * (p22 - p11) > 0 )  //
    {
        std::cout << "The two edges have no intersection." << std::endl;
        return false;
    }
    else if((p11 - p12).transpose() * (p11 - p21) > 0 )
    {
        if((p11 - p12).transpose() * (p22 - p12) > 0)  //
        {
//            std::cout << "Edge 22." << std::endl;
            endPoints = std::make_pair(p21, p22);
        }
        else if((p11 - p12).transpose() * (p12 - p22) > 0)
        {
//            std::cout << "Edge 21." << std::endl;
            endPoints = std::make_pair(p21, p12);
        }
    }
    else if((p11 - p12).transpose() * (p21 - p11) > 0)
    {
        if((p11 - p12).transpose() * (p12 - p22) > 0)  //
        {
//            std::cout << "Edge 11." << std::endl;
            endPoints = std::make_pair(p11, p12);
        }
        else if((p11 - p12).transpose() * (p22 - p12) > 0)
        {
//            std::cout << "Edge 12." << std::endl;
            endPoints = std::make_pair(p11, p22);
        }
    }

    if((endPoints.first - endPoints.second).norm() < edge_legnth_) return false;  //
    return true;
}




void PlaneDetector::DisplayActive(cv::Mat& segColor)
{

    cv::Mat segImage = cv::Mat(depth_img_.rows, depth_img_.cols, CV_32SC1, cv::Scalar(-1));
    for(int i = 0; i < plane_detected.size(); i++)
    {
        for(int k = 0; k < plane_detected[i]->segments_.size(); k++)
        {
            auto points = plane_detected[i]->segments_[k]->measurements_;
            for (auto p: points)
            {
                segImage.at<int>(p.y, p.x) = plane_detected[i]->map_id_;
            }
        }
    }

    cv::Mat img = segImage.clone();
//    segColor = cv::Mat(img.rows, img.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    segColor = color_img_.clone();

    //
    cv::Mat overlay = cv::Mat::zeros(depth_img_.rows, depth_img_.cols, CV_8UC3);
//    cv::Mat overlay = color_img_.clone();
    // Finally return the image
    if(segImage.type() == CV_8UC3)
    {
        segColor = img.clone();
    }
    else
    {
        for (int i = 0; i < img.rows; i++)
        {
            for (int j = 0; j < img.cols; j++)
            {
                int id = (int)img.at<int>(i,j);

                if(id >= 0)
                {
                    double r = colors_bgr[id % 80][0];
                    double g = colors_bgr[id % 80][1];
                    double b = colors_bgr[id % 80][2];
                    //
//                    cv::circle(overlay, cv::Point2f(j, i), 1, cv::Scalar(r, g, b), cv::FILLED);
                    segColor.at<cv::Vec3b>(i,j) = cv::Vec3b(colors_bgr[id % 90]);
                }

            }
        }
    }

    //
//    double alpha = 1.0; // 透明度
//    cv::addWeighted(overlay, 0.5, segColor, 0.5, 0, segColor);


    std::ofstream out_convex_hul;
//    out_convex_hul.open("./planes/" + std::to_string(plane_detected[0]->timestamp_) + "_convex.txt", std::ios::out | std::ios::trunc);

    //
    for(int i = 0; i < plane_detected.size(); i++)
    {

//        // 每个segment单独绘制
//        for(int k = 0; k < plane_detected[i]->segments_.size(); k++)
//        {
//            auto convexs = plane_detected[i]->segments_[k]->meas_convex_;
//            std::vector<cv::Point> hullPoints;
//            for (const auto& point : convexs) {
//                hullPoints.push_back(cv::Point(cvRound(point.x), cvRound(point.y)));
//            }
//            polylines(segColor, hullPoints, true, cv::Scalar(255, 255, 255), 2);
//        }

        //
        std::vector<cv::Point2f> convexHullPoints2D;
        for(int k = 0; k < plane_detected[i]->segments_.size(); k++)
        {
            auto convexs = plane_detected[i]->segments_[k]->meas_convex_;
            for (const auto& point : convexs) {
                convexHullPoints2D.push_back(point);
            }
        }
        std::vector<cv::Point2f> hullPointsNew;
        cv::convexHull(convexHullPoints2D, hullPointsNew, true, true);  //
        std::vector<cv::Point> hullPoints;
        for (const auto& point : hullPointsNew) {
            hullPoints.push_back(cv::Point(cvRound(point.x), cvRound(point.y)));
            out_convex_hul << point.x << " " << point.y << " ";
        }
        out_convex_hul << std::endl;
        polylines(segColor, hullPoints, true, cv::Scalar(255, 255, 255), 2);  //
    }


    //
    for(int i = 0; i < border_detected.size(); i++)
    {
        auto point1 = border_detected[i]->end_points_.first;
        float u1 = point1.x() / point1.z() * K_.at<double>(0, 0) + K_.at<double>(0, 2);
        float v1 = point1.y() / point1.z() * K_.at<double>(1, 1) + K_.at<double>(1, 2);

        auto point2 = border_detected[i]->end_points_.second;
        float u2 = point2.x() / point2.z() * K_.at<double>(0, 0) + K_.at<double>(0, 2);
        float v2 = point2.y() / point2.z() * K_.at<double>(1, 1) + K_.at<double>(1, 2);

        cv::line(segColor, cv::Point2f(u1, v1), cv::Point2f(u2, v2), cv::Scalar(250, 50, 50), 3);
        cv::circle(segColor, cv::Point2f(u1, v1), 5, cv::Scalar(250, 50, 50), cv::FILLED);
        cv::circle(segColor, cv::Point2f(u2, v2), 5, cv::Scalar(250, 50, 50), cv::FILLED);
    }


    //
    for(int i = 0; i < corner_detected.size(); i++)
    {
        Eigen::Vector3d point = corner_detected[i]->position_;
        float u = point.x() / point.z() * K_.at<double>(0, 0) + K_.at<double>(0, 2);
        float v = point.y() / point.z() * K_.at<double>(1, 1) + K_.at<double>(1, 2);

        Eigen::Matrix3d bases = corner_detected[i]->R_M0toC_;
        Eigen::Vector3d norm = corner_detected[i]->line_length_;
        Eigen::Vector3d endpointX = point + bases.col(0) * norm(0);
        Eigen::Vector3d endpointY = point + bases.col(1) * norm(1);
        Eigen::Vector3d endpointZ = point + bases.col(2) * norm(2);
        float uX = endpointX.x() / endpointX.z() * K_.at<double>(0, 0) + K_.at<double>(0, 2);
        float vX = endpointX.y() / endpointX.z() * K_.at<double>(1, 1) + K_.at<double>(1, 2);
        float uY = endpointY.x() / endpointY.z() * K_.at<double>(0, 0) + K_.at<double>(0, 2);
        float vY = endpointY.y() / endpointY.z() * K_.at<double>(1, 1) + K_.at<double>(1, 2);
        float uZ = endpointZ.x() / endpointZ.z() * K_.at<double>(0, 0) + K_.at<double>(0, 2);
        float vZ = endpointZ.y() / endpointZ.z() * K_.at<double>(1, 1) + K_.at<double>(1, 2);

        cv::line(segColor, cv::Point2f(u, v), cv::Point2f(uX, vX), cv::Scalar(0,0,255), 3);  // x
        cv::line(segColor, cv::Point2f(u, v), cv::Point2f(uY, vY), cv::Scalar(0,255,0), 3);  // y
        cv::line(segColor, cv::Point2f(u, v), cv::Point2f(uZ, vZ), cv::Scalar(255,0,0), 3);  // z
        cv::circle(segColor, cv::Point2f(u, v), 5, cv::Scalar(50, 50, 50), cv::FILLED);
    }

    //
    for(int i = 0; i < plane_detected.size(); i++)
    {
        auto plane = plane_detected[i];
        for(int k = 0; k < plane->segments_.size(); k++)
        {
            cv::Point2f center(0,0);
            auto points = plane->segments_[k]->measurements_;
            for (auto p: points)
            {
                center = center + p;
            }
            center.x  = center.x / points.size();
            center.y  = center.y / points.size();
            cv::putText(segColor, std::to_string(plane->id_) + "[" + std::to_string(plane->map_id_) + "]", center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        }
    }

}