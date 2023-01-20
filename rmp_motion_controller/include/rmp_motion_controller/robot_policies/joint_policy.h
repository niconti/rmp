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


namespace rmp {


class JointPolicy : public RobotPolicy {
protected:

  std::shared_ptr<RedundancyPolicy> redundancy_policy;

public:

  JointPolicy(const KDL::Chain &kdl_chain) : RobotPolicy(kdl_chain) { }


  void addTarget(const Eigen::VectorXd &q_goal)
  {
    auto q_rmp = std::make_shared<rmp::RedundancyPolicy>(q_goal);
    redundancy_policy = q_rmp;
  }


  MotionPolicy computeMotionPolicy(const KDL::JntArray &jpos, const KDL::JntArray &jvel)
  {

    /*
     * 1) An RMP X(fi,Ai) is created for each task map, where fi = xi_acc desired; */

    {
      auto q_pos = jpos.data;
      auto q_vel = jvel.data;
      redundancy_policy->update(q_pos,q_vel);
    }

    /*
     * 2) The RMPs are pulled back into the confguration space; */

    /*
     * 3) The pulled back RMPs are summed; */

    const int n = kdl_chain.getNrOfJoints();

    MotionPolicy q_sum;
    q_sum.f = Eigen::VectorXd::Zero(n);
    q_sum.A = Eigen::MatrixXd::Identity(n,n);

    {
      q_sum = q_sum + *redundancy_policy;
    }

    /*
     * 4) The combined RMP is itself pulled back into an unconstrained space to
     *    handle joint limits. */

    return q_sum;
  }


};

}  // namespace
#endif