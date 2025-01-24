//
// Created by zyd on 24-6-13.
//

#include "corner.h"
using namespace Rvg;

Position::Position(int dim) : Vec(dim)
{
}

void Position::Update(const Eigen::VectorXd &dx)
{
    // Update estimate
    assert(dx.rows() == size);
    SetValue(value + dx);
}



Rotation::Rotation():JPLQuat()
{

}

Rotation::~Rotation()
{

}



Corner::Corner() : BasicType(6)
{
    // Initialize subvariables
    q = std::shared_ptr<JPLQuat>(new JPLQuat());
    p = std::shared_ptr<Vec>(new Vec(3));

    // Set our default state value
    Eigen::Matrix<double, 7, 1> pose0;
    pose0.setZero();
    pose0(3) = 1.0;

    SetValue(pose0);
    SetFej(pose0);
}

Corner::~Corner()
{

}

void Corner::SetLocalId(int new_id)
{
    id = new_id;
    q->SetLocalId(new_id);
    p->SetLocalId(new_id + ((new_id!=-1) ? q->GetSize() : 0));
}

void Corner::Update(const Eigen::VectorXd &dx)
{
    assert(dx.rows() == size);

    Eigen::Matrix<double, 7, 1> newX = value;

    Eigen::Matrix<double, 4, 1> dq;
    dq << .5 * dx.block(0, 0, 3, 1), 1.0;
    dq = QuatUtils::QuatNorm(dq);

    //Update orientation
    newX.block(0, 0, 4, 1) =  QuatUtils::QuatMultiply(dq, GetQuat());

    //Update position
    newX.block(4, 0, 3, 1) += dx.block(3, 0, 3, 1);

    SetValue(newX);
}

void Corner::SetValue(const Eigen::MatrixXd& new_value)
{

    assert(new_value.rows() == 7);
    assert(new_value.cols() == 1);

    //Set orientation value
    q->SetValue(new_value.block(0, 0, 4, 1));

    //Set position value
    p->SetValue(new_value.block(4, 0, 3, 1));

    value = new_value;
}

void Corner::SetFej(const Eigen::MatrixXd& new_value)
{
    assert(new_value.rows() == 7);
    assert(new_value.cols() == 1);

    //Set orientation fej value
    q->SetFej(new_value.block(0, 0, 4, 1));

    //Set position fej value
    p->SetFej(new_value.block(4, 0, 3, 1));

    fej = new_value;
}

std::shared_ptr<BasicType> Corner::Clone()
{
    auto Clone = std::shared_ptr<Corner>(new Corner());
    Clone->SetValue(GetValue());
    Clone->SetFej(GetFej());

    return Clone;
}

std::shared_ptr<BasicType> Corner::CheckIfSubvariable(const std::shared_ptr<BasicType> check)
{
    if (check == q) {
        return q;
    } else if (check == p) {
        return p;
    }

    return nullptr;
}

/// Rotation access
Eigen::Matrix<double, 3, 3> Corner::GetRotation() const
{
    return q->GetRotation();
}

/// FEJ Rotation access
Eigen::Matrix<double, 3, 3> Corner::GetRotationFej() const
{
    return q->GetRotationFej();;
}

/// Rotation access as quaternion
Eigen::Matrix<double, 4, 1> Corner::GetQuat() const
{
    return q->GetValue();
}

/// FEJ Rotation access as quaternion
Eigen::Matrix<double, 4, 1> Corner::GetQuatFej() const
{
    return q->GetFej();
}

/// Position access
Eigen::Matrix<double, 3, 1> Corner::GetPos() const
{
    return p->GetValue();
}

// FEJ position access
Eigen::Matrix<double, 3, 1> Corner::GetPosFej() const
{
    return p->GetFej();
}

// Quaternion type access
std::shared_ptr<JPLQuat> Corner::GetQ()
{
    return q;
}

// Position type access
std::shared_ptr<Vec> Corner::GetP()
{
    return p;
}


DataType Corner::ClassofSubvariable()
{
    return POSEJPL;
}