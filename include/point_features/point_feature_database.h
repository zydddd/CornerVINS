/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/11.
 */

#ifndef RGBD_PLANE_VIO_POINT_FEATURE_DATABASE_H
#define RGBD_PLANE_VIO_POINT_FEATURE_DATABASE_H

#include <vector>
#include <mutex>
#include <memory>
#include <Eigen/Eigen>

#include "point_feature.h"

namespace Rvg {

/**
 * @brief PointFeature Database containing features we are currently tracking.
 * Each visual tracker has this database in it and it contains all features that we are tracking.
 * For Multi-Threading safety, we use "remove" label to delete features
 */
class PointFeatureDatabase
{
public:
    /**
     * @brief Default constructor
     */
    PointFeatureDatabase() {}


    /**
     * @brief Get a specified feature by ID
     */
    std::shared_ptr<PointFeature> GetFeature(size_t id, bool remove=false);


    /**
     * @brief Update a feature object
     * This will update a given feature based on the passed ID it has.
     * It will create a new feature, if it is an ID that we have not seen before.
     */
    void UpdateFeature(size_t id, size_t plane_id, double timestamp, float u, float v, float u_n, float v_n, double depth);
    /**
     * @brief Get features that do not have newer measurement then the specified time.
     * This function will return all features that do not a measurement at a time greater than the specified time.
     * For example this could be used to get features that have not been successfully tracked into the newest frame.
     * All features returned will not have any measurements occurring at a time greater then the specified.
     */
    std::vector<std::shared_ptr<PointFeature>> FeaturesNotContainingNewer(double timestamp, bool remove=false,
            bool skip_deleted=false);


    /**
     * @brief Get features that has measurements older then the specified time.
     * This will collect all features that have measurements occurring before the specified timestamp.
     * For example, we would want to remove all features older then the last clone/state in our sliding window.
     */
    std::vector<std::shared_ptr<PointFeature>> FeaturesContainingOlder(double timestamp, bool remove=false,
            bool skip_deleted=false);

    /**
     * @brief Get features that has measurements at the specified time.
     * This function will return all features that have the specified time in them.
     * This would be used to get all features that occurred at a specific clone/state.
     */
    std::vector<std::shared_ptr<PointFeature>> FeaturesContaining(double timestamp, bool remove=false,
            bool skip_deleted=false);

    /**
     * @brief This function will delete all features that have been used up.
     * If a feature was unable to be used, it will still remain since it will not have a delete flag set
     */
    void Cleanup();

    /**
     * @brief This function will delete all feature measurements that are older then the specified timestamp
     */
    void CleanupMeasurements(double timestamp);

    /**
     * @brief Will update the passed database with this database's latest feature information.
     */
    void AppendNewMeasurements(const std::shared_ptr<PointFeatureDatabase>& database);


    /**
     * @brief Returns the size of the feature database
     */
    size_t Size()
    {
        std::unique_lock<std::mutex> lck(mtx_);
        return featuresIdLookup_.size();
    }


    /**
     * @brief Returns the internal data (should not normally be used)
     */
    std::unordered_map<size_t, std::shared_ptr<PointFeature>> GetInternalData()
    {
        std::unique_lock<std::mutex> lck(mtx_);
        return featuresIdLookup_;
    }

protected:

    /// Mutex lock for our map
    std::mutex mtx_;

    /// Lookup array that allow use to query based on ID
    std::unordered_map<size_t, std::shared_ptr<PointFeature>> featuresIdLookup_;
};



}



#endif RGBD_PLANE_VIO_POINT_FEATURE_DATABASE_H
