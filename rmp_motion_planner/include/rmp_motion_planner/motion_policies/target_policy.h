#ifndef RMP_MOTION_PLANNER_MOTION_POLICIES_TARGET_POLICY_H
#define RMP_MOTION_PLANNER_MOTION_POLICIES_TARGET_POLICY_H
#include "rmp_motion_planner/motion_policies.h"
// STL
#include <cmath>
// Eigen
#include <Eigen/Dense>


namespace rmp {


class TargetPolicy : public XMotionPolicy {
private:

  Eigen::Vector3d x_goal;

  double alpha;
  double beta;


  Eigen::Vector3d soft_norm(const Eigen::Vector3d &v)
  {
    double c = 10.0;

    auto h = [&](double z) { return z + (1.0 / c) * std::log(1 + std::exp(-2.0 * c * z)); };

    return v / h(v.norm());
  }


  Eigen::Vector3d policy(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) override
  {
    return alpha * soft_norm(x_goal-x_pos) - beta * x_vel;
  }

  Eigen::Matrix3d metric(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) override
  {
    return Eigen::Matrix3d::Identity();
  }

public:

  TargetPolicy(const Eigen::Vector3d &x_goal, const Eigen::Vector3d &x=Eigen::Vector3d::Zero())
  {
    this->x = x;
    this->x_goal = x_goal;

    alpha = 1.0;
    beta  = 10.0;
  }


  void setGoal(const Eigen::Vector3d &x_goal)
  {
    this->x_goal = x_goal;
  }

};


}  // namespace
#endif
