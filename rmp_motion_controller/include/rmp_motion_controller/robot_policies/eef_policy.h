#ifndef RMP_MOTION_CONTROLLER_ROBOT_POLICIES_EEF_POLICY_H
#define RMP_MOTION_CONTROLLER_ROBOT_POLICIES_EEF_POLICY_H
#include <vector>
#include <memory>
// ROS
#include <eigen_conversions/eigen_kdl.h>
// KDL
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
//
#include "rmp_motion_controller/motion_policies.h"
#include "rmp_motion_controller/motion_policies/target_policy.h"
#include "rmp_motion_controller/robot_policies/link_policy.h"


namespace rmp {


class EndEffectorPolicy : public LinkPolicy {

  std::vector<std::shared_ptr<TargetPolicy>> target_policies;

public:

  EndEffectorPolicy(const KDL::Chain &kdl_chain) : LinkPolicy(kdl_chain) 
  { }


  void addTarget(const Eigen::Vector3d &x_goal)
  {
    Eigen::Vector3d x(0.0, 0.0, 0.0);

    auto x_rmp = std::make_shared<rmp::TargetPolicy>(x_goal,x);
    target_policies.push_back(x_rmp);
  }


  void setTarget(const Eigen::Vector3d &x_goal)
  {
    target_policies[0]->setGoal(x_goal);
  }


  MotionPolicy computeMotionPolicy(const KDL::JntArray &jpos, const KDL::JntArray &jvel)
  {
    KDL::Frame xpos;
    fk_pos_solver->JntToCart(jpos, xpos);
    jacobian_solver->JntToJac(jpos, jacobian);

    /*
     * 1) An RMP X(fi,Ai) is created for each task map, where fi = xi_acc desired; */

    for (const auto &x_rmp : target_policies)
    {
      Eigen::Isometry3d A;
      tf::transformKDLToEigen(xpos, A);

      Eigen::Vector3d x_pos = A * x_rmp->x;
      Eigen::MatrixXd J = jacobian.data.topRows(3);
      Eigen::Vector3d x_vel = J * jvel.data;

      x_rmp->update(x_pos,x_vel);
    }

    for (const auto &x_rmp : collision_policies)
    {
      Eigen::Isometry3d A;
      tf::transformKDLToEigen(xpos, A);

      Eigen::Vector3d x_pos = A * x_rmp->x;
      Eigen::MatrixXd J = jacobian.data.topRows(3);
      Eigen::Vector3d x_vel = J * jvel.data;

      x_rmp->update(x_pos,x_vel);
    }

    /*
     * 2) The RMPs are pulled back into the confguration space; */

    /*
     * 3) The pulled back RMPs are summed; */

    const int n = kdl_chain.getNrOfJoints();

    MotionPolicy q_sum;
    q_sum.f = Eigen::VectorXd::Zero(n);
    q_sum.A = Eigen::MatrixXd::Identity(n,n);

    for (const auto &x_rmp : target_policies)
    {
        Eigen::MatrixXd J = jacobian.data.topRows(3);
        q_sum = q_sum + pullback(*x_rmp, J);
    }

    for (const auto &x_rmp : collision_policies)
    {
        Eigen::MatrixXd J = jacobian.data.topRows(3);
        q_sum = q_sum + pullback(*x_rmp, J);
    }

    /*
    * 4) The combined RMP is itself pulled back into an unconstrained space to
    *    handle joint limits. */

    return q_sum;
  }


};

}  // namespace
#endif