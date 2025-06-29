#ifndef RMP_MOTION_CONTROLLER_MOTION_POLICIES_JOINT_SPEED_LIMIT_POLICY_H
#define RMP_MOTION_CONTROLLER_MOTION_POLICIES_JOINT_SPEED_LIMIT_POLICY_H
#include "rmp_motion_controller/motion_policies.h"
#include <cmath>
// XML-RPC
#include <XmlRpcValue.h>


namespace rmp {


class JointSpeedLimitPolicy : public QMotionPolicy {
private:

  double v_max;
  unsigned int q_index;

  double nu = 100;      // Overall priority weight relative to other RMPs
  double kd = 1000.0;   // Damping gain, determining amount of “drag”
  double vr = 1.5;      // Defines width of velocity region affect by damping


  double sign(double value)
  {
    return value / std::abs(value);
  }


  Eigen::VectorXd policy(const Eigen::VectorXd &q_pos, const Eigen::VectorXd &q_vel) override
  {
    Eigen::VectorXd q_acc = Eigen::VectorXd::Zero(q_pos.size());
    q_acc[q_index] = -kd * sign(q_vel[q_index]) * (std::abs(q_vel[q_index]) - (v_max - vr));
    return q_acc;
  }

  Eigen::MatrixXd metric(const Eigen::VectorXd &q_pos, const Eigen::VectorXd &q_vel) override
  {
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(q_pos.size(),q_pos.size());
    if (std::abs(q_vel[q_index]) < (v_max - vr))
    {
      M(q_index, q_index) = 0;
    }
    else
    {
      M(q_index, q_index) = 1.0 - (std::pow(std::abs(q_vel[q_index]) - (v_max - vr), 2) / std::pow(vr, 2));
    }
    return M;
  }

public:

  JointSpeedLimitPolicy(double v_max, unsigned int q_index)
  {
    this->v_max = v_max;
    this->q_index = q_index;
  }


  void setConfig(const XmlRpc::XmlRpcValue &param)
  {
    nu = param["metric_weight"];
    kd = param["damping_gain"];
    vr = param["velocity_damping_region"];
  }


  void setSpeedLimit(double v_max)
  {
    this->v_max = v_max;
  }

};


}  // namespace
#endif
