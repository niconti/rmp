#ifndef RMP_MOTION_CONTROLLER_MATH_H
#define RMP_MOTION_CONTROLLER_MATH_H
#include <iostream> 
// Eigen
#include <Eigen/Dense>


namespace rmp {


Eigen::MatrixXd pinv(const Eigen::MatrixXd &A);


}  // namespace
#endif
