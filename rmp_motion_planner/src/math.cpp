#include "rmp_motion_planner/math.h"


namespace rmp {


Eigen::MatrixXd pinv(const Eigen::MatrixXd &A)
{
  // std::cout << "AT * A:" << std::endl << A.transpose() * A << std::endl;
  // return (A.transpose() * A).inverse() * A.transpose();
  return A.completeOrthogonalDecomposition().pseudoInverse();
}


// Eigen::MatrixXd pinv(const Eigen::MatrixXd &A)
// {
//   return A.transpose() * (A * A.transpose()).inverse();
// }


}  // namespace
