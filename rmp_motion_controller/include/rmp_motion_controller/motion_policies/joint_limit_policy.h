#ifndef RMP_MOTION_CONTROLLER_MOTION_POLICIES_JOINT_LIMIT_POLICY_H
#define RMP_MOTION_CONTROLLER_MOTION_POLICIES_JOINT_LIMIT_POLICY_H
#include "rmp_motion_controller/motion_policies.h"
#include <cmath>
// XML-RPC
#include <XmlRpcValue.h>


namespace rmp {


class JointLimitPolicy : public QMotionPolicy {
private:

  double q_lower;
  double q_upper;
  unsigned int q_index;

  double nu = 1000;     // Overall priority weight relative to other RMPs
  double lm = 0.01;     // Length scale controlling ramp-up of metric as joint limit is approached
  double em = 1e-3;     // Offset determining x value at which metric diverges
  double vm = 0.01;     // Scale determining rate at which metric increases with velocity in direction of barrier
  double kp = 1.0;      // Position gain, determining how strongly configuration is pulled toward target
  double kd = 200.0;    // Damping gain, determining amount of “drag”
  double lp = 0.1;      // Length scale controlling steepness of position barrier
  double ep = 1e-2;     // Offset limiting divergence of position barrier strength

  std::function<double(double)> x;


  Eigen::VectorXd policy(const Eigen::VectorXd &q_pos, const Eigen::VectorXd &q_vel) override
  {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(q_pos.size());
    q[q_index] = kp / (std::pow(x(q_pos[q_index]),2) / std::pow(lp,2) + ep) - kd * x(q_vel[q_index]);
    return q;
  }

  Eigen::MatrixXd metric(const Eigen::VectorXd &q_pos, const Eigen::VectorXd &q_vel) override
  {
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(q_pos.size(),q_pos.size());
    M(q_index, q_index) = (1.0 - 1.0 / (1.0 + std::exp(-x(q_vel[q_index]) / vm))) * nu / (x(q_pos[q_index]) / lm + em);
    return M;
  }

public:

  JointLimitPolicy(double q_lower, double q_upper, unsigned int q_index)
  {
    this->q_lower = q_lower;
    this->q_upper = q_upper;
    this->q_index = q_index;

    x = [=](double q)
    {
      return (q - q_lower) / (q_upper - q_lower);
    };
  }


  void setConfig(const XmlRpc::XmlRpcValue &param)
  {
    nu = param["nu"];
    lm = param["lm"];
    em = param["em"];
    vm = param["vm"];
    kp = param["kp"];
    kd = param["kd"];
    lp = param["lp"];
    ep = param["ep"];
  }


  void setLimits(const double &q_lower, const double &q_upper)
  {
    this->q_lower = q_lower;
    this->q_upper = q_upper;
  }

};


}  // namespace
#endif
