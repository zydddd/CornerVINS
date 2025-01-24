/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by weihao on 2021/1/11.
 */

#ifndef RVG_VIO_POINTTRIANGULATION_H
#define RVG_VIO_POINTTRIANGULATION_H

#include <unordered_map>
#include "point_feature.h"
#include "quat_utils.h"

#include <memory>

namespace Rvg {

/**
 * @brief Struct which stores all our feature initializer options
 */
struct PointTriangulationParameters
{
    /// If we should use depth in triangulation
    bool use_depth = false;
    /// Minimum distance to accept triangulated features
    double min_dist = 0.10;

    /// Minimum distance to accept triangulated features
    double max_dist = 200;


    /// If we should perform 1d triangulation instead of 3d
    bool triangulate_1d = false;

    /// If we should perform Levenberg-Marquardt refinment
    bool refine_features = true;

    /// Max runs for Levenberg-Marquardt
    int max_runs = 5;

    /// Init lambda for Levenberg-Marquardt optimization
    double init_lamda = 1e-3;

    /// Max lambda for Levenberg-Marquardt optimization
    double max_lamda = 1e10;

    /// Cutoff for dx increment to consider as converged
    double min_dx = 1e-6;

    /// Cutoff for cost decrement to consider as converged
    double min_dcost = 1e-6;

    /// Multiplier to increase/decrease lambda
    double lam_mult = 10;



    /// Max baseline ratio to accept triangulated features
    double max_baseline = 400;

    /// Max condition number of linear triangulation matrix accept triangulated features
    double max_cond_number = 50000;
};


/**
 * @brief Class that triangulates feature
 */
class PointTriangulation
{

public:

    /**
     * @brief Structure which stores pose estimates for use in triangulation
     */
    struct ClonePose
    {
        /// Rotation
        Eigen::Matrix<double,3,3> _Rot;

        /// Position
        Eigen::Matrix<double,3,1> _pos;

        /// Constructs pose from rotation and position
        ClonePose(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &p)
        {
            _Rot = R;
            _pos = p;
        }

        /// Constructs pose from quaternion and position
        ClonePose(const Eigen::Matrix<double,4,1> &q, const Eigen::Matrix<double,3,1> &p)
        {
            _Rot = Rvg::QuatUtils::Quat2Rot(q);
            _pos = p;
        }

        /// Default constructor
        ClonePose()
        {
            _Rot = Eigen::Matrix<double,3,3>::Identity();
            _pos = Eigen::Matrix<double,3,1>::Zero();
        }

        /// Accessor for rotation
        const Eigen::Matrix<double,3,3> &Rot()
        {
            return _Rot;
        }

        /// Accessor for position
        const Eigen::Matrix<double,3,1> &pos()
        {
            return _pos;
        }

    };


    /**
     * @brief Default constructor
     * @param options Options for the initializer
     */
    PointTriangulation(PointTriangulationParameters &options) : options_(options) {}

    /**
     * @brief Uses a linear triangulation to get initial2 estimate for the feature
     */
    bool SingleTriangulation(PointFeature* feat, std::unordered_map<double,ClonePose> &clonesCAM);
    bool SingleTriangulationWithDepth(PointFeature* feat, std::unordered_map<double,ClonePose> &clonesCAM);

    /**
     * @brief Uses a linear triangulation to get initial2 estimate for the feature, treating the anchor observation as a true bearing.
     */
    bool SingleTriangulation1D(PointFeature* feat, std::unordered_map<double,ClonePose> &clonesCAM);

    bool SingleGaussNewton(PointFeature* feat, std::unordered_map<double,ClonePose> &clonesCAM);

    void FeaturesTriangulation(std::vector<std::shared_ptr<PointFeature>>& feature_vec,
                                                    std::unordered_map<double, ClonePose>& clones_cam);
    /**
     * @brief Gets the current configuration of the feature initializer
     * @return Const feature initializer config
     */
    const PointTriangulationParameters Config()
    {
        return options_;
    }


protected:

    /// Contains options for the initializer process
    PointTriangulationParameters options_;

    /**
     * @brief Helper function for the gauss newton method that computes error of the given estimate
     */
    double ComputeError(std::unordered_map<double,ClonePose> &clonesCAM,PointFeature* feat,double alpha,double beta,double rho);
};

}





#endif //RVG_VIO_POINTTRIANGULATION_H
