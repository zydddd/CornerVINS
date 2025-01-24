//
// Created by zyd on 23-12-22.
//

#include <vector>
#include <iostream>
#include <unordered_map>
#include <Eigen/Eigen>
#include <memory>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "quat_utils.h"

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/octree/octree.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>

#ifndef RGBD_PLANE_VIO_UTILS_H
#define RGBD_PLANE_VIO_UTILS_H


bool IsPointInPolygon(const cv::Point2f& testPoint, const std::vector<cv::Point2f>& polygonVertices);

bool DoSegmentsIntersect(const cv::Point2f& p1, const cv::Point2f& q1,
                         const cv::Point2f& p2, const cv::Point2f& q2);

void getPlaneConvexHull(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& normal,
                        std::vector<Eigen::Vector3d>& convexHullPoints3D);

/**
 * @brief
 * @param point
 * @param normal
 * @param d
 * @return
 */
Eigen::Vector3d ProjectPointToPlane(const Eigen::Vector3d& point, const Eigen::Vector3d& normal, double d);

pcl::PointCloud<pcl::PointXYZ>::Ptr vectorToPointCloud(const std::vector<Eigen::Vector3d>& inputVector);
std::vector<Eigen::Vector3d> pointCloudToVector(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

std::vector<Eigen::Vector3d> SamplePoints(const std::vector<Eigen::Vector3d>& points);

void disp(const std::vector<std::vector<int>>& matrix);
void disp(const Eigen::MatrixXd& matrix);

std::vector<Eigen::Vector3d> GetPointsProjected(const std::vector<Eigen::Vector3d>& pointsCloud, const Eigen::Vector3d& normal, double d);
std::vector<Eigen::Vector3d> GetPointsConvexHull(const std::vector<Eigen::Vector3d>& pointsProjected, const Eigen::Vector3d& normal);


void GetPlanePoints2D(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& normal, Eigen::Vector3d& p0, Eigen::Vector3d& p1,
                      std::vector<cv::Point2f>& points2D);

void findMaxCompleteSubgraph(std::vector<std::vector<int>>& graph, std::vector<int>& candidates,
                             std::vector<int>& currentClique, std::vector<int>& maxClique);
bool isCompleteSubgraph(std::vector<std::vector<int>>& graph, const std::vector<int>& clique);

double Compute2DOverlap(std::vector<cv::Point2f>& points2D1, std::vector<cv::Point2f>& points2D2);
double GomputeConvexHullArea(std::vector<cv::Point2f>& convex_hull);
double ComputeMinDistance(std::vector<cv::Point2f>& points2D1, std::vector<cv::Point2f>& points2D2);


bool compareProjection(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& reference, const Eigen::Vector3d& direction);
void sortPointsOnLine(std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& reference, const Eigen::Vector3d& direction);

Eigen::Vector3d computeIntersection(const Eigen::Vector3d& p1, const Eigen::Vector3d& v1,
                                    const Eigen::Vector3d& p2, const Eigen::Vector3d& v2);
bool ComputeEndPoints(const std::vector<Eigen::Vector3d>& ch_points,
                      const Eigen::Vector3d& direction, const Eigen::Vector3d& pointOnLine,
                      std::pair<Eigen::Vector3d, Eigen::Vector3d>& endPoints, double thres);


//bool ComputeIntersectionPoint(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2,
//                              const Eigen::Vector3d& q1, const Eigen::Vector3d& q2,
//                              Eigen::Vector3d& intersection);


/////////////////////
Eigen::Vector3d computeBordersIntersection(const Eigen::Vector3d& p1, const Eigen::Vector3d& v1,
                                           const Eigen::Vector3d& p2, const Eigen::Vector3d& v2);
void SolveMainDirections(Eigen::Vector3d& plane1, Eigen::Vector3d& plane2, Eigen::Vector3d& plane3,
                         Eigen::Matrix<double, 3, 3>& R);

Eigen::Vector3d computePlanesIntersection(const Eigen::Vector3d& n1, const double d1,
                                          const Eigen::Vector3d& n2, const double d2,
                                          const Eigen::Vector3d& n3, const double d3);


std::vector<Eigen::Vector3d> CalculateCubeVertices(Eigen::Vector3d point, Eigen::Vector3d dir1, Eigen::Vector3d dir2, Eigen::Vector3d dir3);  //
#endif //RGBD_PLANE_VIO_UTILS_H
