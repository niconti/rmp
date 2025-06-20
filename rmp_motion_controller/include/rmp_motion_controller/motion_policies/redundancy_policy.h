#ifndef RMP_MOTION_CONTROLLER_MOTION_POLICIES_REDUNDANCY_POLICY_H
#define RMP_MOTION_CONTROLLER_MOTION_POLICIES_REDUNDANCY_POLICY_H
#include "rmp_motion_controller/motion_policies.h"
#include <cmath>
// Eigen
#include <Eigen/Dense>
// XML-RPC
#include <XmlRpcValue.h>


namespace rmp {


class RedundancyPolicy : public QMotionPolicy {
private:

  Eigen::VectorXd q_goal;

  double nu = 50;       // Priority weight relative to other RMPs
  double kp = 100;      // Position gain, determining how strongly configuration is pulled toward target
  double kd = 50;       // Damping gain, determining amount of “drag”
  double theta = 0.5;   // Distance in c-space at which the position correction vector is capped

  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> r;


  Eigen::VectorXd policy(const Eigen::VectorXd &q_pos, const Eigen::VectorXd &q_vel) override
  {
    return kp * r(q_goal-q_pos) - kd * q_vel;
  }

  Eigen::MatrixXd metric(const Eigen::VectorXd &q_pos, const Eigen::VectorXd &q_vel) override
  {
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(q_pos.size(),q_pos.size());
    return nu * I;
  }

public:

  RedundancyPolicy(const Eigen::VectorXd &q_goal, const Eigen::VectorXd &q=Eigen::VectorXd::Zero(6))
  {
    this->q = q;
    this->q_goal = q_goal;

    r = [=](const Eigen::VectorXd &p)
    {
      Eigen::VectorXd result;
      if (p.norm() < theta)
        result = p;
      else
        result = theta * p / p.norm();
      return result;
    };
  }


  void setConfig(const XmlRpc::XmlRpcValue &param)
  {
    nu = param["nu"];
    kp = param["kp"];
    kd = param["kd"];
    theta = param["theta"];
  }


  void setGoal(const Eigen::VectorXd &q_goal)
  {
    this->q_goal = q_goal;
  }

};


}  // namespace
#endif
