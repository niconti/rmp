#ifndef RMP_MOTION_CONTROLLER_MOTION_POLICIES_TARGET_POLICY_H
#define RMP_MOTION_CONTROLLER_MOTION_POLICIES_TARGET_POLICY_H
#include "rmp_motion_controller/motion_policies.h"
#include <cmath>
#include <functional>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
// XML-RPC
#include <XmlRpcValue.h>


namespace rmp {

class TargetPolicy : public XMotionPolicy {
private:

  Eigen::Vector3d x_goal;

  double nu_near = 10000;     // Metric scalar for the isotropic M_near contribution to the metric (inertia matrix).
  double nu_far  = 2500;      // Metric scalar for the directional M_far contribution to the metric (inertia matrix).
  double kp = 30.0;           // Position gain.
  double kd = 85.0;           // Damping gain.
  double e  = 0.075;          // Length scale controlling transition between constant acceleration region far from target and linear region near target.
  double oa = 0.05;           // Length scale of the Gaussian controlling blending between S and I.
  double a_min = 0.01;        // Controls the minimum contribution of the isotropic M_near term to the metric (inertia matrix).
  double b = 20.0;            // Scale factor controlling the strength of boosting near the target.
  double ob = 0.02;           // Length scale of Gaussian controlling boosting near the target.

  std::function<double(const Eigen::Vector3d&)> alpha;
  std::function<double(const Eigen::Vector3d&)> beta;


  Eigen::Vector3d v(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_obs)
  {
    return x_pos - x_obs;
  }

  Eigen::Vector3d v_norm(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_obs)
  {
    return v(x_pos,x_obs) / v(x_pos,x_obs).norm();
  }


  double dist(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_obs)
  {
    return v(x_pos,x_obs).norm();
  }


  Eigen::Vector3d policy(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) override
  {
    return kp * (x_goal-x_pos) / ((x_goal-x_pos).norm() + e) - kd * x_vel;
  }

  Eigen::Matrix3d metric(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) override
  {
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d M_near = nu_near * I;
    Eigen::Matrix3d S = 1.0 / std::pow((x_goal-x_pos).norm(),2) * (x_goal-x_pos) * (x_goal-x_pos).transpose();
    Eigen::Matrix3d M_far  = nu_far * S;

    return (beta(x_pos) * b + (1 - beta(x_pos))) * (alpha(x_pos) * M_near + (1 - alpha(x_pos)) * M_far);
  }

public:

  TargetPolicy(const Eigen::Vector3d &x_goal, const Eigen::Vector3d &x=Eigen::Vector3d::Zero())
  {
    this->x = x;
    this->x_goal = x_goal;

    alpha = [=](const Eigen::Vector3d &x)
    {
      return (1.0 - a_min) * std::exp(-std::pow((x_goal-x).norm(),2) / (2.0 * std::pow(oa,2))) + a_min;
    };

    beta = [=](const Eigen::Vector3d &x)
    {
      return std::exp(-std::pow((x_goal-x).norm(),2) / (2.0 * std::pow(oa,2)));
    };
  }


  void setConfig(const XmlRpc::XmlRpcValue &param)
  {
    kp = param["kp"];
    kd = param["kd"];
    e  = param["e"];
    oa = param["oa"];
    a_min   = param["a_min"];
    nu_near = param["nu_near"];
    nu_far  = param["nu_far"];
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
