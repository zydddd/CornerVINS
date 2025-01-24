/*
 * RVG-VIO: VIO System construct by NLPR RVG Group, Institute of Automation, CAS
 *
 * Author: Fulin Tang, Hao Wei, Yihong Wu
 * Email: fulin.tang@nlpr.ia.ac.cn
 *        weihao2019@ia.ac.cn
 *
 * Created by Fulin Tang on 2021/1/11.
 */
#ifndef VEC_H
#define VEC_H

#include "basic_type.h"

namespace Rvg {

/**
 * @brief The class implements vector variables based on BasicType
 */
class Vec : public BasicType
{
public:

    /**
     * @brief Default constructor for Vec
     */
    Vec(int dim);

    ~Vec();

    /**
     * @brief Implements the update operation through standard vector addition
     */
    void Update(const Eigen::VectorXd& dx) override;

    /**
     * @brief Performs all the cloning
     */
    std::shared_ptr<BasicType> Clone() override;

};


}




#endif // VEC_H
