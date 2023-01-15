#include "rmp_motion_planner/motion_policies.h"


namespace rmp {


MotionPolicy MotionPolicy::operator+(const MotionPolicy &other)
{
  Eigen::VectorXd f1 = f;
  Eigen::MatrixXd A1 = A;

  Eigen::VectorXd f2 = other.f;
  Eigen::MatrixXd A2 = other.A;

  MotionPolicy rmp;
  rmp.f = pinv(A1 + A2) * (A1*f1 + A2*f2);
  rmp.A = A1 + A2;

  return rmp;
}


// QMotionPolicy QMotionPolicy::operator+(const QMotionPolicy &other)
// {
//   Eigen::VectorXd h1 = h;
//   Eigen::MatrixXd B1 = B;
//
//   Eigen::VectorXd h2 = other.h;
//   Eigen::MatrixXd B2 = other.B;
//
//   QMotionPolicy rmp;
//   rmp.h = pinv(B1 + B2) * (B1*h1 + B2*h2);
//   rmp.B = B1 + B2;
//
//   return rmp;
// }


}  // namespace
