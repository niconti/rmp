#ifndef RMP_MOTION_CONTROLLER_MOTION_POLICIES_TARGET_AXIS_POLICY_H
#define RMP_MOTION_CONTROLLER_MOTION_POLICIES_TARGET_AXIS_POLICY_H
#include "rmp_motion_controller/motion_policies.h"
#include <cmath>
#include <functional>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
// XML-RPC
#include <XmlRpcValue.h>


namespace rmp {

/** 
 *  @brief    Axis Target RPM
 *  @details  Drives x-, y-, or z axis of end effector frame toward target orientation. This RPM is used for general
 *            orientation targets (where an axis target RPM is added for each of three axes) as well as for "partial pose" targets
 *            where only alignment of a single axis is desired.
 */
class TargetAxisPolicy : public XMotionPolicy {
private:

  Eigen::Vector3d x_goal;

  double nu = 100;         // Priority weight relaive to other RMPs.
  double kp = 210.0;      // Position gain.
  double kd = 60.0;       // Damping gain.
  double b  = 3000.0;     // Scale factor controlling the strength of boosting near the position target.
  double ob = 0.08;       // Length scale of the Gaussian controlling boosting near the position target.

  std::function<double(const Eigen::Vector3d&)> beta;


  Eigen::Vector3d n(const Eigen::Vector3d &x)
  {
    return x / x.norm();
  }

  Eigen::Vector3d n_dot(const Eigen::Vector3d &x_dot)
  {
    return x_dot / x_goal.norm();
  }


  Eigen::Vector3d policy(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) override
  {
    Eigen::Vector3d n0 = n(x_goal);
    Eigen::Vector3d n_pos = n(x_pos);
    // Eigen::Vector3d n_vel = n_dot(x_vel);
    Eigen::Vector3d n_vel = x_vel / (x_pos.norm());
    Eigen::Vector3d n_acc = kp * (n0 - n_pos) - kd * n_vel;
    return n_acc;
  }

  Eigen::Matrix3d metric(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) override
  {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d M_boosted = (beta(x_pos) * b + (1 - beta(x_pos))) * nu * I;
    Eigen::Matrix3d M = nu * I;
    return M;
  }

public:

  TargetAxisPolicy(const Eigen::Vector3d &x_goal, const Eigen::Vector3d &x=Eigen::Vector3d::Zero())
  {
    this->x = x;
    this->x_goal = x_goal;

    beta = [=](const Eigen::Vector3d &x)
    {
      return std::exp(-std::pow((x_goal-x).norm(),2) / (2.0 * std::pow(ob,2)));
    };
  }


  void setConfig(const XmlRpc::XmlRpcValue &param)
  {
    nu = param["nu"];
    kp = param["kp"];
    kd = param["kd"];
    b  = param["b"];
    ob = param["ob"];
  }


  void setGoal(const Eigen::Vector3d &x_goal)
  {
    this->x_goal = x_goal;
  }

};


}  // namespace
#endif