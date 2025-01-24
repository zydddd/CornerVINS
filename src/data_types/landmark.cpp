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

using namespace Rvg;

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