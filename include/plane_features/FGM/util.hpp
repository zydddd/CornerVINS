#ifndef UTIL_HPP_INCLUDED
#define UTIL_HPP_INCLUDED

#include <Eigen/Dense>


Eigen::MatrixXd resize(const Eigen::MatrixXd& m, int rows, int cols, double default_value);

Eigen::MatrixXd mat2ind(const Eigen::MatrixXd& m);

Eigen::MatrixXd mat2indC(const Eigen::MatrixXd& m);

Eigen::MatrixXd multDiag(const Eigen::MatrixXd& A, const Eigen::VectorXd& v);

int sub2ind(int dimrow, int dimcol, int row, int col);

Eigen::VectorXi sub2ind(int dimrow, int dimcol, const Eigen::VectorXi setrow, const Eigen::VectorXi setcol);

template <typename Derived>
Eigen::VectorXi find(const Eigen::DenseBase<Derived>& m);

Eigen::MatrixXd normalize_bistochastic(const Eigen::MatrixXd& X, double tol, int max_iters);

#endif
