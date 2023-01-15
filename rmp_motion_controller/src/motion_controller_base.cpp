#include "rmp_motion_controller/motion_controller_base.h"

namespace rmp {


template <class HardwareInterface>
MotionPolicy MotionControllerBase<HardwareInterface>::pullback(const XMotionPolicy &x_rmp, const Eigen::MatrixXd &J)
{
  Eigen::Vector3d f = x_rmp.f;
  Eigen::Matrix3d A = x_rmp.A;

  std::cout << "XRMP.f:" << std::endl << x_rmp.f << std::endl;
  std::cout << "XRMP.A:" << std::endl << x_rmp.A << std::endl;

  std::cout << "J:" << std::endl << J << std::endl;

  MotionPolicy q_rmp;
  q_rmp.f = pinv(J.transpose() * A * J) * J.transpose() * A * f;
  q_rmp.A = J.transpose() * A * J;

  std::cout << "QRMP.f:" << std::endl << q_rmp.f << std::endl;
  std::cout << "QRMP.A:" << std::endl << q_rmp.A << std::endl;

  return q_rmp;
}

template <class HardwareInterface>
MotionPolicy MotionControllerBase<HardwareInterface>::pushforward(const QMotionPolicy &q_rmp, const Eigen::MatrixXd &J)
{
  Eigen::VectorXd h = q_rmp.f;
  Eigen::MatrixXd B = q_rmp.A;

  MotionPolicy x_rmp;
  x_rmp.f = J * h;
  x_rmp.A = pinv(J).transpose() * B * pinv(J);
  return x_rmp;
}


template <class HardwareInterface>
void MotionControllerBase<HardwareInterface>::addXMotionPolicy(const std::shared_ptr<rmp::XMotionPolicy> &rmp)
{
  x_motion_policies.push_back(rmp);
}

template <class HardwareInterface>
void MotionControllerBase<HardwareInterface>::addQMotionPolicy(const std::shared_ptr<rmp::QMotionPolicy> &rmp)
{
  q_motion_policies.push_back(rmp);
}


template <class HardwareInterface>
void MotionControllerBase<HardwareInterface>::clearXMotionPolicies()
{
  x_motion_policies.clear();
}

template <class HardwareInterface>
void MotionControllerBase<HardwareInterface>::clearQMotionPolicies()
{
  q_motion_policies.clear();
}


template <class HardwareInterface>
void MotionControllerBase<HardwareInterface>::computeMotionPolicies()
{
  jacobian_solver->JntToJac(current_jpos, jacobian);

  /*
   * 1) An RMP X(fi,Ai) is created for each task map, where fi = xi_acc desired; */

  for (const auto &x_rmp : x_motion_policies)
  {
    fk_pos_solver->JntToCart(current_jpos, current_xpos);

    Eigen::Isometry3d A;
    // tf::transformKDLToEigen(current_xpos, A);

    Eigen::Vector3d x_pos = A * x_rmp->x;

    Eigen::MatrixXd J = jacobian.data.topRows(3);

    Eigen::Vector3d x_vel = J * q_vel;

    x_rmp->update(x_pos,x_vel);
  }

  for (const auto &q_rmp : q_motion_policies)
  {
    q_rmp->update(q_pos,q_vel);
  }

  /*
   * 2) The RMPs are pulled back into the confguration space; */

  /*
   * 3) The pulled back RMPs are summed; */

  const int n = q_acc.rows();

  MotionPolicy q_sum;
  q_sum.f = Eigen::VectorXd::Zero(n);
  q_sum.A = Eigen::MatrixXd::Identity(n,n);

  for (const auto &x_rmp : x_motion_policies)
  {
    Eigen::MatrixXd J = jacobian.data.topRows(3);

    q_sum = q_sum + pullback(*x_rmp, J);
  }

  for (const auto &q_rmp : q_motion_policies)
  {
    q_sum = q_sum + *q_rmp;
  }

  /*
   * 4) The combined RMP is itself pulled back into an unconstrained space to
   *    handle joint limits. */

   q_acc = q_sum.f;
}


template class MotionControllerBase<hardware_interface::PositionJointInterface>;
template class MotionControllerBase<hardware_interface::VelocityJointInterface>;
}