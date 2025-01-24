/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/11.
 */

#ifndef RGBD_PLANE_VIO_POINT_FEATURE_H
#define RGBD_PLANE_VIO_POINT_FEATURE_H

#include <vector>
#include <iostream>
#include <unordered_map>
#include <Eigen/Eigen>


namespace Rvg {


/**
 * @brief Point Feature Class/Struct
 * The life-long tracking information of the feature
 * */
class PointFeature
{
public:

    /// ID of this feature
    size_t feat_id;

    /// If this feature should be deleted
    bool to_delete;

    /// Image coordinates that this feature has been seen from
    std::vector<Eigen::VectorXf> uvs;
    /// Image normalized coordinates that this feature has been seen from
    std::vector<Eigen::VectorXf> uvs_norm;
    /// Image depth that this feature has been seen from
    std::vector<double> depths;
    /// Timestamps of each UV measurement
    std::vector<double> timestamps;
    std::vector<size_t> plane_ids;  // plane ID of this feature

    /// Timestamp of anchor clone
    double anchor_clone_timestamp;

    /// Triangulated position of this feature, in the anchor frame
    Eigen::Vector3d p_FinA;
    /// Triangulated position of this feature, in the global frame
    Eigen::Vector3d p_FinG;


    /**
     * @brief Remove measurements that do not occur at input timestamps.
     * Given a series of valid timestamps, this will remove all measurements that have not occurred at these times.
     * @param valid_times Vector of timestamps that our measurements must occur at
     */
    void CleanOldMeasurements(std::vector<double> valid_times);

    /**
     * @brief Remove measurements that are older then the input timestamp.
     *
     * Given a valid timestamp, this will remove all measurements that have occured earlier then this.
     * Include equal time measurement
     *
     * @param timestamp Timestamp that our measurements must occur after
     */
    void CleanOlderMeasurements(double timestamp);

    /**
     * @brief Get feature's by a special timestamp
     * @param timestamp
     * @param uv
     * @param uv_norm
     * */
    bool GetMeasurementByTime(double timestamp, Eigen::VectorXf& uv, Eigen::VectorXf& uv_norm);
     bool GetMeasurementByTime(double timestamp, Eigen::VectorXf& uv, Eigen::VectorXf& uv_norm, double& d);
};


}



#endif RGBD_PLANE_VIO_POINT_FEATURE_H
