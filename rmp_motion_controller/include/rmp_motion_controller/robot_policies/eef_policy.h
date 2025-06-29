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
#include <kdl/chainjnttojacdotsolver.hpp>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
//
#include "rmp_motion_controller/motion_policies.h"
#include "rmp_motion_controller/motion_policies/target_policy.h"
#include "rmp_motion_controller/motion_policies/target_axis_policy.h"
#include "rmp_motion_controller/robot_policies/link_policy.h"

#define UNIT_VECTOR_SCALE   1.0;


namespace rmp {


class EndEffectorPolicy : public LinkPolicy {
private:
  std::shared_ptr<TargetPolicy> target_policy;
  std::shared_ptr<TargetAxisPolicy> x_axis_policy;
  std::shared_ptr<TargetAxisPolicy> y_axis_policy;
  std::shared_ptr<TargetAxisPolicy> z_axis_policy;

public:

  EndEffectorPolicy(const KDL::Chain &kdl_chain) : LinkPolicy(kdl_chain) 
  { }


  void addTarget(const Eigen::Isometry3d &X_goal)
  {
    Eigen::Vector3d o_goal = X_goal.translation();
    target_policy = std::make_shared<rmp::TargetPolicy>(o_goal);

    Eigen::Matrix3d R_goal = X_goal.rotation();
    Eigen::Vector3d ux = Eigen::Vector3d::UnitX() * UNIT_VECTOR_SCALE;
    Eigen::Vector3d uy = Eigen::Vector3d::UnitY() * UNIT_VECTOR_SCALE;
    Eigen::Vector3d uz = Eigen::Vector3d::UnitZ() * UNIT_VECTOR_SCALE;
    x_axis_policy = std::make_shared<rmp::TargetAxisPolicy>(R_goal*ux,ux);
    // y_axis_policy = std::make_shared<rmp::TargetAxisPolicy>(R_goal*uy,uy);
    // z_axis_policy = std::make_shared<rmp::TargetAxisPolicy>(R_goal*uz,uz);
  }


  void setTarget(const Eigen::Isometry3d &X_goal)
  {
    Eigen::Vector3d o_goal = X_goal.translation();
    if (target_policy) target_policy->setGoal(o_goal);

    Eigen::Matrix3d R_goal = X_goal.rotation();
    Eigen::Vector3d ux = Eigen::Vector3d::UnitX() * UNIT_VECTOR_SCALE;
    Eigen::Vector3d uy = Eigen::Vector3d::UnitY() * UNIT_VECTOR_SCALE;
    Eigen::Vector3d uz = Eigen::Vector3d::UnitZ() * UNIT_VECTOR_SCALE;
    // std::cout << "x_axis:\n" << X_goal*ux << std::endl;
    // std::cout << "y_axis:\n" << X_goal*uy << std::endl;
    // std::cout << "z_axis:\n" << X_goal*uz << std::endl;
    if (x_axis_policy) x_axis_policy->setGoal(R_goal*ux);
    if (y_axis_policy) y_axis_policy->setGoal(R_goal*uy);
    if (z_axis_policy) z_axis_policy->setGoal(R_goal*uz);
  }


  MotionPolicy computeMotionPolicy(const KDL::JntArray &jpos, const KDL::JntArray &jvel) override
  {
    KDL::Frame xpos;
    fk_pos_solver->JntToCart(jpos, xpos);
    jacobian_solver->JntToJac(jpos, jacobian);
    // jacobian_dot_solver->JntToJacDot(jpos, jacobian_dot);

    /*
     * 1) An RMP X(fi,Ai) is created for each task map, where fi = xi_acc desired; */

    if (target_policy)
    {
      auto x_rmp = target_policy;
      Eigen::Isometry3d A;
      tf::transformKDLToEigen(xpos, A);
      Eigen::Vector3d x_pos = A * x_rmp->x;

      KDL::Vector point;
      tf::vectorEigenToKDL(x_rmp->x, point);
      KDL::Jacobian jacobian_offset = jacobian;
      jacobian_offset.changeRefPoint(point);
      Eigen::MatrixXd J = jacobian_offset.data.topRows(3);
      Eigen::Vector3d x_vel = J * jvel.data;

      x_rmp->update(x_pos,x_vel);
    }

    if (x_axis_policy)
    {
      auto x_rmp = x_axis_policy;
      Eigen::Isometry3d A;
      tf::transformKDLToEigen(xpos, A);
      Eigen::Vector3d x_pos = A.rotation() * x_rmp->x;

      KDL::Vector point;
      tf::vectorEigenToKDL(x_rmp->x, point);
      KDL::Jacobian jacobian_offset = jacobian;
      jacobian_offset.changeRefPoint(point);
      Eigen::MatrixXd J = jacobian_offset.data.topRows(3);
      Eigen::Vector3d x_vel = J * jvel.data;

      x_rmp->update(x_pos,x_vel);
    }

    if (y_axis_policy)
    {
      auto x_rmp = y_axis_policy;
      Eigen::Isometry3d A;
      tf::transformKDLToEigen(xpos, A);
      Eigen::Vector3d x_pos = A.rotation() * x_rmp->x;

      KDL::Vector point;
      tf::vectorEigenToKDL(x_rmp->x, point);
      KDL::Jacobian jacobian_offset = jacobian;
      jacobian_offset.changeRefPoint(point);
      Eigen::MatrixXd J = jacobian_offset.data.topRows(3);
      Eigen::Vector3d x_vel = J * jvel.data;

      x_rmp->update(x_pos,x_vel);
    }

    if (z_axis_policy)
    {
      auto x_rmp = z_axis_policy;
      Eigen::Isometry3d A;
      tf::transformKDLToEigen(xpos, A);
      Eigen::Vector3d x_pos = A.rotation() * x_rmp->x;

      KDL::Vector point;
      tf::vectorEigenToKDL(x_rmp->x, point);
      KDL::Jacobian jacobian_offset = jacobian;
      jacobian_offset.changeRefPoint(point);
      Eigen::MatrixXd J = jacobian_offset.data.topRows(3);
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

    XMotionPolicy x_sum;
    x_sum.f = Eigen::Vector3d::Zero();
    x_sum.A = Eigen::Matrix3d::Identity();

    if (target_policy)
    {
      auto x_rmp = target_policy;
      KDL::Vector point;
      tf::vectorEigenToKDL(x_rmp->x, point);

      KDL::Jacobian jacobian_offset = jacobian;
      jacobian_offset.changeRefPoint(point);
      Eigen::MatrixXd J = jacobian_offset.data.topRows(3);
      // std::cout << "pullback_trg:\n";
      // std::cout << pullback(*x_rmp, J).f << std::endl;

      q_sum = q_sum + pullback(*x_rmp, J);
      x_sum = x_sum + *x_rmp;
    }

    if (x_axis_policy)
    {
      auto x_rmp = x_axis_policy;
      KDL::Vector point;
      tf::vectorEigenToKDL(x_rmp->x, point);

      KDL::Jacobian jacobian_offset = jacobian;
      jacobian_offset.changeRefPoint(point);
      Eigen::MatrixXd J = jacobian_offset.data.topRows(3);
      // std::cout << "pullback_x_axis:\n";
      // std::cout << pullback(*x_rmp, J).f << std::endl;
      
      q_sum = q_sum + pullback(*x_rmp, J);
      x_sum = x_sum + *x_rmp;
    }

    if (y_axis_policy)
    {
      auto x_rmp = y_axis_policy;
      KDL::Vector point;
      tf::vectorEigenToKDL(x_rmp->x, point);

      KDL::Jacobian jacobian_offset = jacobian;
      jacobian_offset.changeRefPoint(point);
      Eigen::MatrixXd J = jacobian_offset.data.topRows(3);
      // std::cout << "pullback_y_axis:\n";
      // std::cout << pullback(*x_rmp, J).f << std::endl;
      
      q_sum = q_sum + pullback(*x_rmp, J);
      x_sum = x_sum + *x_rmp;
    }

    if (z_axis_policy)
    {
      auto x_rmp = z_axis_policy;
      KDL::Vector point;
      tf::vectorEigenToKDL(x_rmp->x, point);

      KDL::Jacobian jacobian_offset = jacobian;
      jacobian_offset.changeRefPoint(point);
      Eigen::MatrixXd J = jacobian_offset.data.topRows(3);
      // std::cout << "pullback_z_axis:\n";
      // std::cout << pullback(*x_rmp, J).f << std::endl;

      q_sum = q_sum + pullback(*x_rmp, J);
      x_sum = x_sum + *x_rmp;
    }

    // for (const auto &x_rmp : collision_policies)
    // {
    //   Eigen::MatrixXd J = jacobian.data.topRows(3);
    //   q_sum = q_sum + pullback(*x_rmp, J);
    // }

    /*
     * 4) The combined RMP is itself pulled back into an unconstrained space to
     *    handle joint limits. */

    // Eigen::MatrixXd J = jacobian.data.topRows(3);
    // q_sum = pullback(x_sum, J);
    return q_sum;
  }


};

}  // namespace
#endif