#ifndef RMP_MOTION_PLANNER_MOTION_POLICIES_REDUNDANCY_POLICY_H
#define RMP_MOTION_PLANNER_MOTION_POLICIES_REDUNDANCY_POLICY_H
#include "rmp_motion_planner/motion_policies.h"
// STL
#include <cmath>
// Eigen
#include <Eigen/Dense>


namespace rmp {


class RedundancyPolicy : public QMotionPolicy {
private:

  Eigen::VectorXd q_goal;

  double alpha;
  double beta;


  Eigen::VectorXd policy(const Eigen::VectorXd &q_pos, const Eigen::VectorXd &q_vel) override
  {
    return alpha * (q_goal-q_pos) - beta * q_vel;
  }

  Eigen::MatrixXd metric(const Eigen::VectorXd &q_pos, const Eigen::VectorXd &q_vel) override
  {
    return Eigen::MatrixXd::Identity(q_pos.size(),q_pos.size());
  }

public:

  RedundancyPolicy(const Eigen::VectorXd &q_goal, const Eigen::VectorXd &q=Eigen::VectorXd::Zero(6))
  {
    this->q = q;
    this->q_goal = q_goal;

    alpha = 0.1;
    beta  = 0.2;
  }


  void setGoal(const Eigen::Vector3d &q_goal)
  {
    this->q_goal = q_goal;
  }

};


}  // namespace
#endif
