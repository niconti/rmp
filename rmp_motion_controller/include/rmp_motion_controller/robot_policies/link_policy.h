#ifndef RMP_MOTION_CONTROLLER_ROBOT_POLICIES_LINK_POLICY_H
#define RMP_MOTION_CONTROLLER_ROBOT_POLICIES_LINK_POLICY_H
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
#include "rmp_motion_controller/robot_policies.h"
#include "rmp_motion_controller/motion_policies/collision_policy.h"


namespace rmp {


class LinkPolicy : public RobotPolicy {
protected:

  int link_index = -1;

  KDL::Jacobian jacobian;

  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
  std::shared_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;

  std::shared_ptr<KDL::ChainJntToJacSolver> jacobian_solver;

  std::vector<std::shared_ptr<CollisionPolicy>> collision_policies;

public:

  LinkPolicy(const KDL::Chain &kdl_chain, int link_index = -1) : RobotPolicy(kdl_chain)
  {
    this->link_index = link_index;

    fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
    fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain));

    const int n_joints = kdl_chain.getNrOfJoints();
    jacobian.resize(n_joints);   
    jacobian_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));
  }


  void addObstacle(const Eigen::Vector3d &x_obs)
  {
    Eigen::Vector3d x(0.0, 0.0, 0.0);

    auto x_rmp = std::make_shared<rmp::CollisionPolicy>(x_obs,x);
    collision_policies.push_back(x_rmp);
  }


  void setObstacle(const Eigen::Vector3d &x_obs)
  {
    for (const auto &policy : collision_policies)
    {
      policy->setObstacle(x_obs);
    }
  }


  MotionPolicy computeMotionPolicy(const KDL::JntArray &jpos, const KDL::JntArray &jvel)
  {
    KDL::Frame xpos;
    fk_pos_solver->JntToCart(jpos, xpos, link_index);
    jacobian_solver->JntToJac(jpos, jacobian, link_index);

    /*
     * 1) An RMP X(fi,Ai) is created for each task map, where fi = xi_acc desired; */

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