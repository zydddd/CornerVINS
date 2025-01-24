#ifndef FGM_HPP_INCLUDED
#define FGM_HPP_INCLUDED

#include <utility>
#include <string>
#include <map>
#include <Eigen/Dense>

#include "util.hpp"
#include "hungarian.hpp"

#define FGM_VERSION "0.0.2"


Eigen::MatrixXd gmPosDHun(Eigen::MatrixXd& X);

double multGXHSQTr(const Eigen::MatrixXd& indG, const Eigen::MatrixXd& X,
                   const Eigen::MatrixXd& indH, const Eigen::MatrixXd& IndS0, const Eigen::MatrixXd& Q);

std::pair<Eigen::MatrixXd, double> fgm(Eigen::MatrixXd& KP, Eigen::MatrixXd& KQ, 
                                       Eigen::MatrixXd& Ct, Eigen::MatrixXd& asgTX, 
                                       std::map<std::string, Eigen::MatrixXd>& gph1,
                                       std::map<std::string, Eigen::MatrixXd>& gph2,
                                       int nAlp = 101, int nItMa = 100, int nHst = 10);

#endif
