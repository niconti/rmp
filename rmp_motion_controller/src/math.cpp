#include "rmp_motion_controller/math.h"


namespace rmp {


Eigen::MatrixXd pinv(const Eigen::MatrixXd &A)
{
  // return (A.transpose() * A).inverse() * A.transpose();
  return A.completeOrthogonalDecomposition().pseudoInverse();
}


// Eigen::MatrixXd pinv(const Eigen::MatrixXd &A)
// {
//   return A.transpose() * (A * A.transpose()).inverse();
// }


}  // namespace
