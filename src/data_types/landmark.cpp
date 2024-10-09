/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by Fulin Tang on 2021/1/11.
 */
#include "landmark.h"

using namespace RvgVio;


Landmark::Landmark(int dim) : Vec(dim)
{

}


void Landmark::Update(const Eigen::VectorXd &dx)
{
    // Update estimate
    assert(dx.rows() == size);
    SetValue(value + dx);
    // Ensure we are not near zero in the z-direction
    if (LandmarkRepresentation::IsRelativeRepresentation(feat_representation) && value(value.rows()-1) < 1e-8) {
        std::cerr << "ERROR: the depth became close to zero in update" << std::endl;
        should_marg = true;
    }
}

Eigen::Matrix<double,3,1> Landmark::GetXYZ(bool getfej) const
{

    /// CASE: Global 3d feature representation
    /// CASE: Anchored 3D feature representation
    if (feat_representation == LandmarkRepresentation::Representation::GLOBAL_3D) {
        return (getfej) ? GetFej() : GetValue();
    }

    /// CASE: Global inverse depth feature representation
    /// CASE: Anchored full inverse depth feature representation
    if (feat_representation == LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH) {
        Eigen::Matrix<double, 3, 1> p_invFinG = (getfej) ? GetFej() : GetValue();
        Eigen::Matrix<double, 3, 1> p_FinG;
        p_FinG << (1 / p_invFinG(2)) * std::cos(p_invFinG(0)) * std::sin(p_invFinG(1)),
                  (1 / p_invFinG(2)) * std::sin(p_invFinG(0)) * std::sin(p_invFinG(1)),
                  (1 / p_invFinG(2)) * std::cos(p_invFinG(1));
        return p_FinG;
    }

    // Failure
    assert(false);

    return Eigen::Vector3d::Zero();
}



void Landmark::SetFromXYZ(Eigen::Matrix<double,3,1> p_FinG, bool isfej)
{

    /// CASE: Global 3d feature representation
    /// CASE: Anchored 3d feature representation
    if (feat_representation == LandmarkRepresentation::Representation::GLOBAL_3D) {
        if(isfej) {
            SetFej(p_FinG);
        } else {
            SetValue(p_FinG);
        }

        return;
    }

    /// CASE: Global inverse depth feature representation
    /// CASE: Anchored inverse depth feature representation
    if (feat_representation == LandmarkRepresentation::Representation::ANCHORED_FULL_INVERSE_DEPTH) {
        // Feature inverse representation
        // NOTE: This is not the MSCKF inverse form, but the standard form
        // NOTE: Thus we go from p_FinG and convert it to this form
        double g_rho = 1/p_FinG.norm();
        double g_phi = std::acos(g_rho*p_FinG(2));
        //double g_theta = std::asin(g_rho*p_FinG(1)/std::sin(g_phi));
        double g_theta = std::atan2(p_FinG(1),p_FinG(0));
        Eigen::Matrix<double,3,1> p_invFinG;
        p_invFinG(0) = g_theta;
        p_invFinG(1) = g_phi;
        p_invFinG(2) = g_rho;

        // Set our feature value
        if(isfej) {
            SetFej(p_invFinG);
        } else {
            SetValue(p_invFinG);
        }

        return;
    }

    // Failure
    assert(false);
}

DataType Landmark::ClassofSubvariable()
{
    return LANDMARK;
}

PlaneCP::PlaneCP(int dim) : Vec(dim)
{

}

void PlaneCP::Update(const Eigen::VectorXd &dx)
{
    // Update estimate
    assert(dx.rows() == size);
    SetValue(value + dx);
}

StructurePlane::StructurePlane(int dim) : Vec(dim)
{

}

void StructurePlane::Update(const Eigen::VectorXd &dx)
{
    // Update estimate
    assert(dx.rows() == size);
    SetValue(value + dx);
}