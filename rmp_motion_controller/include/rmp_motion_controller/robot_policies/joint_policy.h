#ifndef RMP_MOTION_CONTROLLER_ROBOT_POLICIES_JOINT_POLICY_H
#define RMP_MOTION_CONTROLLER_ROBOT_POLICIES_JOINT_POLICY_H
#include <vector>
#include <memory>
// ROS
#include <eigen_conversions/eigen_kdl.h>
// KDL
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
//
#include "rmp_motion_controller/robot_policies.h"
#include "rmp_motion_controller/motion_policies/redundancy_policy.h"
#include "rmp_motion_controller/motion_policies/joint_limit_policy.h"
#include "rmp_motion_controller/motion_policies/joint_speed_limit_policy.h"


namespace rmp {


class JointPolicy : public RobotPolicy {
protected:

  std::shared_ptr<RedundancyPolicy> redundancy_policy;
  std::vector<std::shared_ptr<JointLimitPolicy>> joint_limit_policies;
  std::vector<std::shared_ptr<JointSpeedLimitPolicy>> joint_speed_limit_policies;

public:

  JointPolicy(const KDL::Chain &kdl_chain) : RobotPolicy(kdl_chain) 
  { 

  }


  void addTarget(const Eigen::VectorXd &q_goal)
  {
    auto q_rmp = std::make_shared<RedundancyPolicy>(q_goal);
    redundancy_policy = q_rmp;
  }


  void addJointLimits(double q_lower, double q_upper, int q_index)
  {
    auto q_rmp = std::make_shared<JointLimitPolicy>(q_lower, q_upper, q_index);
    joint_limit_policies.push_back(q_rmp);
  }


  void addJointSpeedLimit(double v_max, int q_index)
  {
    auto q_rmp = std::make_shared<JointSpeedLimitPolicy>(v_max, q_index);
    joint_speed_limit_policies.push_back(q_rmp);
  }


  MotionPolicy computeMotionPolicy(const KDL::JntArray &jpos, const KDL::JntArray &jvel)
  {

    /*
     * 1) An RMP X(fi,Ai) is created for each task map, where fi = xi_acc desired; */

    if (redundancy_policy)
    {
      auto q_pos = jpos.data;
      auto q_vel = jvel.data;
      redundancy_policy->update(q_pos,q_vel);
    }

    for (int i = 0; i < joint_limit_policies.size(); i++)
    {
      auto q_pos = jpos.data;
      auto q_vel = jvel.data;
      joint_limit_policies[i]->update(q_pos,q_vel);
    }

    for (int i = 0; i < joint_speed_limit_policies.size(); i++)
    {
      auto q_pos = jpos.data;
      auto q_vel = jvel.data;
      joint_speed_limit_policies[i]->update(q_pos,q_vel);
    }

    /*
     * 2) The RMPs are pulled back into the confguration space; */

    /*
     * 3) The pulled back RMPs are summed; */

    const int n = kdl_chain.getNrOfJoints();

    MotionPolicy q_sum;
    q_sum.f = Eigen::VectorXd::Zero(n);
    q_sum.A = Eigen::MatrixXd::Identity(n,n);

    if (redundancy_policy)
    {
      q_sum = q_sum + *redundancy_policy;
    }

    for (int i = 0; i < joint_limit_policies.size(); i++)
    {
      // std::cout << i <<", joint_limit_qsum:\n";
      // std::cout << joint_limit_policies[i]->f << std::endl;
      // std::cout << i <<", joint_limit_qsum:\n";
      // std::cout << joint_limit_policies[i]->A << std::endl;
      q_sum = q_sum + *joint_limit_policies[i];
    }

    for (int i = 0; i < joint_speed_limit_policies.size(); i++)
    {
      q_sum = q_sum + *joint_speed_limit_policies[i];
    }

    /*
     * 4) The combined RMP is itself pulled back into an unconstrained space to
     *    handle joint limits. */

    return q_sum;
  }


};

}  // namespace
#endif