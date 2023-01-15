#ifndef RMP_MOTION_CONTROLLER_MOTION_CONTROLLER_BASE_H
#define RMP_MOTION_CONTROLLER_MOTION_CONTROLLER_BASE_H
#include <cmath>
#include <string>
#include <vector>
#include <memory>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
// KDL
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
// Eigen
#include <Eigen/Dense>
//
#include "rmp_motion_controller/math.h"
#include "rmp_motion_controller/motion_policies.h"
#include "rmp_motion_controller/motion_policies/target_policy.h"
#include "rmp_motion_controller/motion_policies/collision_policy.h"


namespace rmp {

template <class HardwareInterface>
class MotionControllerBase : public controller_interface::Controller<HardwareInterface> {
protected:


  KDL::JntArray current_jpos;
  KDL::JntArray current_jvel;
  KDL::JntArray current_jacc;

  KDL::Tree kdl_tree;
  KDL::Chain kdl_chain;

  KDL::Frame current_xpos;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver;
  std::shared_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver;

  KDL::Jacobian jacobian;
  std::shared_ptr<KDL::ChainJntToJacSolver> jacobian_solver;


  Eigen::VectorXd q_pos;
  Eigen::VectorXd q_vel;
  Eigen::VectorXd q_acc;

  std::vector<std::shared_ptr<XMotionPolicy>> x_motion_policies;
  std::vector<std::shared_ptr<QMotionPolicy>> q_motion_policies;

  MotionPolicy pullback(const XMotionPolicy &x_rmp, const Eigen::MatrixXd &J);
  MotionPolicy pushforward(const QMotionPolicy &q_rmp, const Eigen::MatrixXd &J);

  void addXMotionPolicy(const std::shared_ptr<rmp::XMotionPolicy> &rmp);
  void addQMotionPolicy(const std::shared_ptr<rmp::QMotionPolicy> &rmp);

  void clearXMotionPolicies();
  void clearQMotionPolicies();

  void computeMotionPolicies();

};


} // namespace
#endif