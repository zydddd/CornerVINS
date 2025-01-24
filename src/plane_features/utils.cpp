//
// Created by zyd on 23-12-22.
//

#include "utils.h"

using namespace Rvg;

//
bool ArePoints1On2SameSide(const std::vector<Eigen::Vector3d>& cv1, const Eigen::Vector3d& n1, const double d1,
                           const std::vector<Eigen::Vector3d>& cv2, const Eigen::Vector3d& n2, const double d2)
{
    double d0 = n2.transpose()  * cv1[0] - d2;
    for(auto p1 : cv1)
    {
        double dis = n2.transpose()  * p1 - d2;  //
        if(dis * d0 < 0)
            return false;
    }
    return true;
}

//
double Distance1to2(const std::vector<Eigen::Vector3d>& cv1, const Eigen::Vector3d& n1, const double d1,
                    const std::vector<Eigen::Vector3d>& cv2, const Eigen::Vector3d& n2, const double d2)
{
    double res = 1e9;
    for(auto p1 : cv1)
    {
        double dis = abs(n2.transpose()  * p1 - d2);
        if(dis < res)
            res = dis;
    }
    return res;
}

double Angle(const Eigen::Vector3d& n1, const Eigen::Vector3d& n2)
{
    return acosf(std::min(1.0,abs(n1.transpose() * n2)));  // 0-pi/2
}

double MinVerticalDistanceOfTwoConvexHulls(const std::vector<Eigen::Vector3d>& cv1, const Eigen::Vector3d& n1, const double d1,
                                           const std::vector<Eigen::Vector3d>& cv2, const Eigen::Vector3d& n2, const double d2)
{
    if(ArePoints1On2SameSide(cv1,n1, d1, cv2, n2, d2) && ArePoints1On2SameSide(cv2,n2, d2, cv1, n1, d1))
    {
        double d12 = Distance1to2(cv1,n1, d1, cv2, n2, d2);
        double d21 = Distance1to2(cv2,n2, d2, cv1, n1, d1);
        return std::min(d12,d21);
    }
    else
    {
        return 0;
    }
}

Eigen::Vector3d ProjectPointToPlane(const Eigen::Vector3d& point, const Eigen::Vector3d& normal, double d)
{
    double distance = normal.dot(point) - d;
    return point - distance * normal / normal.squaredNorm();
}


//
std::vector<Eigen::Vector3d> GetPointsProjected(const std::vector<Eigen::Vector3d>& pointsCloud, const Eigen::Vector3d& normal, double d)
{
    std::vector<Eigen::Vector3d> pointsProjected;
    pointsProjected.clear();

    for(int k = 0; k < pointsCloud.size(); k++)
    {
        Eigen::Vector3d p = pointsCloud[k];
        pointsProjected.push_back(ProjectPointToPlane(p, normal, d));
    }
    return pointsProjected;
}

//
std::vector<Eigen::Vector3d> GetPlaneConvexHull(const std::vector<Eigen::Vector3d>& pointsProjected, const Eigen::Vector3d& normal)
{
    std::vector<Eigen::Vector3d> convexHullPoints3D;
    convexHullPoints3D.clear();

    int n = pointsProjected.size();
    if(n <= 2)
    {
        for (int i = 0; i < n; i++)
        {
            Eigen::Vector3d p_plane = pointsProjected[i];
            convexHullPoints3D.push_back(p_plane);
        }
        return convexHullPoints3D;
    }

    //
    std::vector<cv::Point2f> points2D;
    points2D.clear();

    Eigen::Vector3d z_axis = normal;
    z_axis = z_axis / z_axis.norm();

    Eigen::Vector3d x_axis = pointsProjected[0] - pointsProjected[1];
    // Make x_axis perpendicular to z
    x_axis = x_axis - z_axis * z_axis.transpose() * x_axis;
    x_axis = x_axis / x_axis.norm();
    // Get z from the cross product of these two
    Eigen::Matrix<double,3,1> y_axis = QuatUtils::SkewX(z_axis) * x_axis;
    y_axis = y_axis / y_axis.norm();

    // From these axes get rotation
    Eigen::Matrix<double,3,3> R_plane_to_cam;
    R_plane_to_cam.block(0,0,3,1) = x_axis;
    R_plane_to_cam.block(0,1,3,1) = y_axis;
    R_plane_to_cam.block(0,2,3,1) = z_axis;
    Eigen::Matrix<double,3,1> plane_in_cam = pointsProjected[0];

    for (int i = 0; i < n; i++)
    {
        Eigen::Vector3d p_plane = R_plane_to_cam.transpose() * pointsProjected[i] - R_plane_to_cam.transpose() * plane_in_cam;
        cv::Point2f p_plane_2D(p_plane(0),p_plane(1));  //
        points2D.push_back(p_plane_2D);
    }

    //
    std::vector<cv::Point2f> convexHullPoints2D;
    cv::convexHull(points2D, convexHullPoints2D, true, true);

    //
    for (const auto &point2D: convexHullPoints2D) {
        Eigen::Vector3d point3D(point2D.x, point2D.y, 0);
        Eigen::Vector3d ch_cam = R_plane_to_cam * point3D + plane_in_cam;
        convexHullPoints3D.push_back(ch_cam);
    }

    return convexHullPoints3D;
}

double MaxOverlapOfTwoConvexHulls(const std::vector<Eigen::Vector3d>& cv1, const Eigen::Vector3d& n1, const double d1,
                                  const std::vector<Eigen::Vector3d>& cv2, const Eigen::Vector3d& n2, const double d2)
{
    double o12 = Overlap1to2(cv1,n1, d1, cv2, n2, d2);
    double o21 = Overlap1to2(cv2,n2, d2, cv1, n1, d1);

    return std::max(o12, o21);
}

double Overlap1to2(const std::vector<Eigen::Vector3d>& cv1, const Eigen::Vector3d& n1, const double d1,
                   const std::vector<Eigen::Vector3d>& cv2, const Eigen::Vector3d& n2, const double d2)
{
    //
    std::vector<Eigen::Vector3d> cv1to2 = GetPointsProjected(cv1, n2, d2);

    //
    Eigen::Vector3d p0 = cv2[0];
    Eigen::Vector3d p1 = cv2[1];

    //
    std::vector<cv::Point2f> points2D1, points2D2;
    ProjectedPoints2D(cv1to2, n2, p0, p1, points2D1);
    ProjectedPoints2D(cv2, n2, p0, p1, points2D2);

    return Compute2DOverlapArea(points2D1, points2D2);

}


void ProjectedPoints2D(const std::vector<Eigen::Vector3d>& points, //
                       const Eigen::Vector3d& normal, const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                       std::vector<cv::Point2f>& points2D)  //
{
    int n = points.size();

    points2D.clear();

    Eigen::Vector3d z_axis = normal;
    z_axis = z_axis / z_axis.norm();

    Eigen::Vector3d x_axis = p0 - p1;
    // Make x_axis perpendicular to z
    x_axis = x_axis - z_axis * z_axis.transpose() * x_axis;
    x_axis = x_axis / x_axis.norm();
    // Get z from the cross product of these two
    Eigen::Matrix<double,3,1> y_axis = QuatUtils::SkewX(z_axis) * x_axis;
    y_axis = y_axis / y_axis.norm();

    // From these axes get rotation
    Eigen::Matrix<double,3,3> R_plane_to_cam;
    R_plane_to_cam.block(0,0,3,1) = x_axis;
    R_plane_to_cam.block(0,1,3,1) = y_axis;
    R_plane_to_cam.block(0,2,3,1) = z_axis;
    Eigen::Matrix<double,3,1> plane_in_cam = p0;


    for (int i = 0; i < n; i++)
    {
        Eigen::Vector3d p_plane = R_plane_to_cam.transpose() * points[i] - R_plane_to_cam.transpose() * plane_in_cam;
        cv::Point2f p_plane_2D(p_plane(0),p_plane(1));  //
        points2D.push_back(p_plane_2D);
    }
}

//
double Compute2DOverlapArea(const std::vector<cv::Point2f>& points2D1, const std::vector<cv::Point2f>& points2D2)  //
{
    int n1 = points2D1.size();
    int n2 = points2D2.size();


    if(n1 < 3 || n2 < 3)
    {
        return -1.0 * Compute2DSeparationDistance(points2D1, points2D2);
    }


    ///////////// 交集区域由落在凸包内部的角点和两凸包边的交点围成
    /// 计算内部点
    std::vector<cv::Point2f> corners;
    for(auto p1: points2D1){
        if (IsPointInPolygon(p1, points2D2))
            corners.push_back(p1);
    }
    for(auto p2: points2D2){
        if (IsPointInPolygon(p2, points2D1))
            corners.push_back(p2);
    }
    /// 寻找两个多边形的交点
    // 遍历第一个多边形的边
    for (std::size_t i = 0; i < n1; i++)
    {
        const cv::Point2f& p1 = points2D1[i];
        const cv::Point2f& q1 = points2D1[(i + 1) % n1];

        // 遍历第二个多边形的边
        for (std::size_t j = 0; j < n2; ++j) {
            const cv::Point2f& p2 = points2D2[j];
            const cv::Point2f& q2 = points2D2[(j + 1) % n2];

            // 检查边是否相交
            if (IsSegmentsIntersect(p1, q1, p2, q2))
            {
                // 计算交点
                float x1 = p1.x, y1 = p1.y;
                float x2 = q1.x, y2 = q1.y;
                float x3 = p2.x, y3 = p2.y;
                float x4 = q2.x, y4 = q2.y;
                float denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
                float px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator;
                float py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator;
                corners.emplace_back(px, py);
            }
        }
    }


    /// 无内部点且无交点，说明两个平面的凸包相离，计算他们的距离
    if(corners.size() < 3)
    {
        // 计算最小距离 TODO 相离情况
        return -1.0 * Compute2DSeparationDistance(points2D1, points2D2);
    }

    // 将交集多边形求凸包，进而求面积
    std::vector<cv::Point2f> IntersectionCHPoints2D;
    cv::convexHull(corners, IntersectionCHPoints2D, true, true);

    ///////////////// 计算面积
    return ComputeConvexHullArea2D(IntersectionCHPoints2D);
}


//
bool IsPointInPolygon(const cv::Point2f& testPoint, const std::vector<cv::Point2f>& polygonVertices)
{
    int vertexCount = polygonVertices.size();
    bool inside = false;

    for (int i = 0, j = vertexCount - 1; i < vertexCount; j = i++) {
        const cv::Point2f& vertex1 = polygonVertices[i];
        const cv::Point2f& vertex2 = polygonVertices[j];

        //
        if ((vertex1.y > testPoint.y) != (vertex2.y > testPoint.y) &&
            testPoint.x < (vertex2.x - vertex1.x) * (testPoint.y - vertex1.y) / (vertex2.y - vertex1.y) + vertex1.x) {
            inside = !inside;
        }
    }

    return inside;
}

//
bool IsSegmentsIntersect(const cv::Point2f& p1, const cv::Point2f& q1,
                         const cv::Point2f& p2, const cv::Point2f& q2)
{
    ///
    auto ccw = [](const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) {
        return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x);
    };

    return ccw(p1, p2, q2) != ccw(q1, p2, q2) && ccw(p1, q1, p2) != ccw(p1, q1, q2);

}

double ComputeConvexHullArea2D(const std::vector<cv::Point2f>& ch)
{
    double totalArea = 0.0; //
    //
    cv::Point2f basePoint = ch[0];

    //
    for (int i = 1; i < ch.size() - 1; i++)
    {
        Eigen::Vector2d v1(ch[i].x - basePoint.x, ch[i].y - basePoint.y);
        Eigen::Vector2d v2(ch[i+1].x - basePoint.x, ch[i+1].y - basePoint.y);
        //
        double area = 0.5 * std::abs(v1.x() * v2.y() - v2.x() * v1.y());
        //
        totalArea += area;
    }

    return totalArea;
}


double Compute2DSeparationDistance(const std::vector<cv::Point2f>& points2D1, const std::vector<cv::Point2f>& points2D2)
{


    int n1 = points2D1.size();
    int n2 = points2D2.size();

    double sep = std::numeric_limits<double>::max();

    if(n1 < 3 || n2 < 3)
    {
        //
        for (const auto& point1 : points2D1) {
            for (const auto& point2 : points2D2) {
                double distance = cv::norm(point1 - point2);
                if (distance < sep)
                    sep = distance;
            }
        }
        return sep;
    }


    ///
    // 1
    cv::Point2f center1;
    for(auto p1: points2D1){
        center1.x+=p1.x;
        center1.y+=p1.y;
    }
    center1 = center1 / n1;
    // 2
    cv::Point2f center2;
    for(auto p2: points2D2){
        center2.x+=p2.x;
        center2.y+=p2.y;
    }
    center2 = center2 / n2;

    //
    if(IsPointInPolygon(center1, points2D2) || IsPointInPolygon(center2, points2D1))
        return 0;

    ///
    // 1
    cv::Point2f inter1;
    for (std::size_t j = 0; j < n1; ++j){
        const cv::Point2f& p1 = points2D1[j];
        const cv::Point2f& q1= points2D1[(j + 1) % n1];

        //
        if (IsSegmentsIntersect(p1, q1, center1, center2))
        {
            //
            float x1 = p1.x, y1 = p1.y;
            float x2 = q1.x, y2 = q1.y;
            float x3 = center1.x, y3 = center1.y;
            float x4 = center2.x, y4 = center2.y;
            float denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            float px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator;
            float py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator;

            inter1.x = px;
            inter1.y = py;
            break;
        }
    }

    // 2
    cv::Point2f inter2;
    for (std::size_t j = 0; j < n2; ++j){
        const cv::Point2f& p2 = points2D2[j];
        const cv::Point2f& q2= points2D2[(j + 1) % n2];

        //
        if (IsSegmentsIntersect(p2, q2, center1, center2))
        {
            //
            float x1 = p2.x, y1 = p2.y;
            float x2 = q2.x, y2 = q2.y;
            float x3 = center1.x, y3 = center1.y;
            float x4 = center2.x, y4 = center2.y;
            float denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            float px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator;
            float py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator;

            inter2.x = px;
            inter2.y = py;
            break;
        }
    }


    Eigen::Vector2d c1(center1.x, center1.y);
    Eigen::Vector2d i1(inter1.x,inter1.y);
    Eigen::Vector2d i2(inter2.x,inter2.y);
    if((c1-i2).norm() > (c1-i1).norm())
    {
        std::cout << "4" << std::endl;
        std::cout << (i1-i2).norm() << std::endl;
        return (i1-i2).norm();
    }
    else
    {
        return 0.0;
    }
}

std::vector<Eigen::Vector3d> SamplePointClouds(const std::vector<Eigen::Vector3d>& points)
{
    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& v : points) {
        pcl::PointXYZ point;
        point.x = static_cast<float>(v.x());
        point.y = static_cast<float>(v.y());
        point.z = static_cast<float>(v.z());
        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;    //
    sor.setInputCloud(cloud);             //
    sor.setLeafSize(0.02f, 0.02f, 0.02f); //
    sor.filter(*sampledCloud);          //

    //
    std::vector<Eigen::Vector3d> sampled_points;
    sampled_points.clear();
    for (const auto& point : sampledCloud->points) {
        Eigen::Vector3d v(point.x, point.y, point.z);
        sampled_points.push_back(v);
    }
    return sampled_points;
}

void disp(const std::vector<std::vector<int>>& matrix)
{
    if(matrix.size() == 0)
    {
        std::cout << "[]" << std::endl;
        return;
    }

    int node1 = matrix.size();
    for(int i=0;i<node1;i++)
    {
        int node2 = matrix[i].size();
        for(int j=0;j<node2;j++)
        {
            std::cout << matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void disp(const Eigen::MatrixXd& matrix)
{
    if(matrix.size() == 0)
    {
        std::cout << "[]" << std::endl;
        return;
    }

    int node1 = matrix.rows();
    int node2 = matrix.cols();
    for(int i=0;i<node1;i++)
    {
        for(int j=0;j<node2;j++)
        {
            std::cout << matrix(i,j) << " ";
        }
        std::cout << std::endl;
    }
}

bool isCompleteSubgraph(std::vector<std::vector<int>>& graph, const std::vector<int>& clique) {
    for (int i = 0; i < clique.size(); ++i) {
        for (int j = i + 1; j < clique.size(); ++j) {
            if (graph[clique[i]][clique[j]] == 0 || graph[clique[j]][clique[i]] == 0) {
                return false;
            }
        }
    }
    return true;
}

void findMaxCompleteSubgraph(std::vector<std::vector<int>>& graph, std::vector<int>& candidates,
                             std::vector<int>& currentClique, std::vector<int>& maxClique) {
    if (candidates.empty()) {
        if (currentClique.size() > maxClique.size() && isCompleteSubgraph(graph, currentClique)) {
            maxClique = currentClique;
        }
        return;
    }

    int node = candidates.back();
    candidates.pop_back();

    currentClique.push_back(node);
    findMaxCompleteSubgraph(graph, candidates, currentClique, maxClique);
    currentClique.pop_back();

    findMaxCompleteSubgraph(graph, candidates, currentClique, maxClique);

    candidates.push_back(node);
}