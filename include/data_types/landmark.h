/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by Fulin Tang on 2021/1/11.
 */

#ifndef LANDMARK_H
#define LANDMARK_H

#include "vec.h"
#include "jpl_quat.h"
#include <iostream>

namespace Rvg {

/**
 * @brief Class has useful feature representation types
 */
    class LandmarkRepresentation
    {

    public:

        /**
         * @brief What feature representation our state can use
         */
        enum Representation
        {
            GLOBAL_3D,
            ANCHORED_FULL_INVERSE_DEPTH,
            UNKNOWN
        };


        /**
         * @brief Returns a string representation of this enum value.
         */
        static inline std::string AsString(Representation feat_representation)
        {

            if(feat_representation==GLOBAL_3D)
                return "GLOBAL_3D";
            if(feat_representation==ANCHORED_FULL_INVERSE_DEPTH)
                return "ANCHORED_FULL_INVERSE_DEPTH";

            return "UNKNOWN";
        }

        /**
         * @brief Returns a string representation of this enum value.
         */
        static inline Representation FromString(const std::string& feat_representation)
        {
            if(feat_representation=="GLOBAL_3D")
                return GLOBAL_3D;
            if(feat_representation=="ANCHORED_FULL_INVERSE_DEPTH")
                return ANCHORED_FULL_INVERSE_DEPTH;

            return UNKNOWN;
        }

        /**
         * @brief Helper function that checks if the passed feature representation is a relative or global
         */
        static inline bool IsRelativeRepresentation(Representation feat_representation)
        {
            return (feat_representation == Representation::ANCHORED_FULL_INVERSE_DEPTH);
        }

    private:

        /**
         * All function in this class should be static.
         * Thus an instance of this class cannot be created.
         */
        LandmarkRepresentation() {};

    };


    class Landmark : public Vec
    {
    public:
        Landmark(int dim);

        /// Feature ID of this landmark (corresponds to frontend id)
        size_t feat_id;

        /// Timestamp of anchor clone
        double anchor_clone_timestamp = -1;

        /// Boolean if this landmark has had at least one anchor change
        bool has_had_anchor_change = false;

        /// Boolean if this landmark should be marginalized out
        bool should_marg = false;

        /// First normalized uv coordinate bearing of this measurement (used for single depth representation)
        Eigen::Vector3d uv_norm_zero;

        /// First estimate normalized uv coordinate bearing of this measurement (used for single depth representation)
        Eigen::Vector3d uv_norm_zero_fej;

        /// What feature representation this feature currently has
        LandmarkRepresentation::Representation feat_representation;

        /**
         * @brief Overrides the default vector update rule
         */
        void Update(const Eigen::VectorXd& dx) override;

        /**
         * @brief Will return the position of the feature in the global frame of reference.
         */
        Eigen::Matrix<double,3,1> GetXYZ(bool getfej) const;

        /**
         * @brief Will set the current value based on the representation.
         */
        void SetFromXYZ(Eigen::Matrix<double,3,1> p_FinG, bool isfej);

        DataType ClassofSubvariable() override;

    };


    class PlaneCP: public Vec
    {
    public:
        PlaneCP(int dim);
        void Update(const Eigen::VectorXd& dx) override;
    };

    class StructurePlane: public Vec
    {
    public:
        StructurePlane(int dim);
        void Update(const Eigen::VectorXd& dx) override;
        int d = 1;
    };
}






#endif // LANDMARK_H
