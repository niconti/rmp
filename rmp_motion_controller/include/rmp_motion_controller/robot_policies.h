#ifndef RMP_MOTION_CONTROLLER_ROBOT_POLICIES_H
#define RMP_MOTION_CONTROLLER_ROBOT_POLICIES_H
#include <vector>
#include <memory>
// KDL
#include <kdl/chain.hpp>
//
#include "rmp_motion_controller/math.h"
#include "rmp_motion_controller/motion_policies.h"


namespace rmp {


class RobotPolicy {
protected:

  KDL::Chain kdl_chain;

  RobotPolicy(const KDL::Chain &kdl_chain) : kdl_chain(kdl_chain) { }


  MotionPolicy pullback(const XMotionPolicy &x_rmp, const Eigen::MatrixXd &J)
  {
    Eigen::Vector3d f = x_rmp.f;
    Eigen::Matrix3d A = x_rmp.A;

    MotionPolicy q_rmp;
    q_rmp.f = pinv(J.transpose() * A * J) * J.transpose() * A * f;
    q_rmp.A = J.transpose() * A * J;

    return q_rmp;
  }

  MotionPolicy pushforward(const QMotionPolicy &q_rmp, const Eigen::MatrixXd &J)
  {
    Eigen::VectorXd h = q_rmp.f;
    Eigen::MatrixXd B = q_rmp.A;

    MotionPolicy x_rmp;
    x_rmp.f = J * h;
    x_rmp.A = pinv(J).transpose() * B * pinv(J);
    return x_rmp;
  }


  virtual MotionPolicy computeMotionPolicy(const KDL::JntArray &jpos, const KDL::JntArray &jvel) = 0;

};

}  // namespace
#endif