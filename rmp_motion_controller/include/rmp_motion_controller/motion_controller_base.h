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
#include <eigen_conversions/eigen_kdl.h>
// KDL
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
// Eigen
#include <Eigen/Dense>
//
#include "rmp_motion_controller/robot_policies/joint_policy.h"
#include "rmp_motion_controller/robot_policies/link_policy.h"
#include "rmp_motion_controller/robot_policies/eef_policy.h"


namespace rmp {

template <class HardwareInterface>
class MotionControllerBase : public controller_interface::Controller<HardwareInterface> {
protected:

  //
  KDL::JntArray current_jpos;
  KDL::JntArray current_jvel;
  KDL::JntArray current_jacc;
  
  KDL::Tree kdl_tree;
  KDL::Chain kdl_chain;

  //
  Eigen::VectorXd q_pos;
  Eigen::VectorXd q_vel;
  Eigen::VectorXd q_acc;

  std::shared_ptr<JointPolicy> joint_policy;
  std::vector<std::shared_ptr<LinkPolicy>> link_policies;
  std::shared_ptr<EndEffectorPolicy> eef_policy;


  void computeMotionPolicies()
  {
    KDL::JntArray jvel;
    jvel.resize(7);
    for (int i=0; i < q_vel.size(); i++)
    {
      jvel(i) = q_vel(i);
    }
    computeMotionPolicies(current_jpos, jvel);
  }

private:

  void computeMotionPolicies(const KDL::JntArray &jpos, const KDL::JntArray &jvel)
  {
    /*
     * 2) The RMPs are pulled back into the confguration space; */

    /*
     * 3) The pulled back RMPs are summed; */

    const int n = kdl_chain.getNrOfJoints();

    MotionPolicy q_sum;
    q_sum.f = Eigen::VectorXd::Zero(n);
    q_sum.A = Eigen::MatrixXd::Identity(n,n);

    for (const auto &link_policy : link_policies)
    {
      q_sum = q_sum + link_policy->computeMotionPolicy(jpos, jvel);
    }
    {
      q_sum = q_sum + eef_policy->computeMotionPolicy(jpos, jvel);
    }
    {
      q_sum = q_sum + joint_policy->computeMotionPolicy(jpos, jvel);
    }

    /*
     * 4) The combined RMP is itself pulled back into an unconstrained space to
     *    handle joint limits. */

    q_acc = q_sum.f;
  }

};


} // namespace
#endif