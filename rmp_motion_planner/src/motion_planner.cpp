#include "rmp_motion_planner/motion_planner.h"


namespace rmp {


MotionPolicy MotionPlanner::pullback(const XMotionPolicy &x_rmp, const Eigen::MatrixXd &J)
{
  Eigen::Vector3d f = x_rmp.f;
  Eigen::Matrix3d A = x_rmp.A;

  std::cout << "XRMP.f:" << std::endl << x_rmp.f << std::endl;
  std::cout << "XRMP.A:" << std::endl << x_rmp.A << std::endl;

  std::cout << "J:" << std::endl << J << std::endl;

  MotionPolicy q_rmp;
  q_rmp.f = pinv(J.transpose() * A * J) * J.transpose() * A * f;
  q_rmp.A = J.transpose() * A * J;

  std::cout << "QRMP.f:" << std::endl << q_rmp.f << std::endl;
  std::cout << "QRMP.A:" << std::endl << q_rmp.A << std::endl;

  return q_rmp;
}


MotionPolicy MotionPlanner::pushforward(const QMotionPolicy &q_rmp, const Eigen::MatrixXd &J)
{
  Eigen::VectorXd h = q_rmp.f;
  Eigen::MatrixXd B = q_rmp.A;

  MotionPolicy x_rmp;
  x_rmp.f = J * h;
  x_rmp.A = pinv(J).transpose() * B * pinv(J);
  return x_rmp;
}


void MotionPlanner::addXMotionPolicy(const std::shared_ptr<rmp::XMotionPolicy> &rmp)
{
  x_motion_policies.push_back(rmp);
}

void MotionPlanner::addQMotionPolicy(const std::shared_ptr<rmp::QMotionPolicy> &rmp)
{
  q_motion_policies.push_back(rmp);
}


void MotionPlanner::clearXMotionPolicies()
{
  x_motion_policies.clear();
}

void MotionPlanner::clearQMotionPolicies()
{
  q_motion_policies.clear();
}


void MotionPlanner::computeMotionPolicies()
{
  auto robot_state = planning_component->getStartState();
  auto planning_group_name = planning_component->getPlanningGroupName();

  // robot_state->copyJointGroupPositions(planning_group_name, q_pos);
  // robot_state->copyJointGroupVelocities(planning_group_name, q_vel);
  // robot_state->copyJointGroupAccelerations(planning_group_name, q_acc);

  // std::cout << "q_pos:" << std::endl << q_pos << std::endl;
  // std::cout << "q_vel:" << std::endl << q_vel << std::endl;
  // std::cout << "q_acc:" << std::endl << q_acc << std::endl;

  /*
   * 1) An RMP X(fi,Ai) is created for each task map, where fi = xi_acc desired; */

  for (const auto &x_rmp : x_motion_policies)
  {
    Eigen::Isometry3d A = robot_state->getGlobalLinkTransform(eef_frame);

    Eigen::Vector3d x_pos = A * x_rmp->x;

    std::cout << "x_pos:" << std::endl << x_pos << std::endl;

    Eigen::MatrixXd J = robot_state->getJacobian(robot_model->getJointModelGroup(planning_group_name), x_rmp->x).topRows(3);

    Eigen::Vector3d x_vel = J * q_vel;

    x_rmp->update(x_pos,x_vel);
  }

  for (const auto &q_rmp : q_motion_policies)
  {
    q_rmp->update(q_pos,q_vel);
  }

  /*
   * 2) The RMPs are pulled back intp the confguration space; */

  /*
   * 3) The pulled back RMPs are summed; */

  MotionPolicy q_sum;
  q_sum.f = Eigen::VectorXd::Zero(6);
  q_sum.A = Eigen::MatrixXd::Identity(6,6);

  for (const auto &x_rmp : x_motion_policies)
  {
    Eigen::MatrixXd J = robot_state->getJacobian(robot_model->getJointModelGroup(planning_group_name), x_rmp->x).topRows(3);

    q_sum = q_sum + pullback(*x_rmp, J);
  }

  for (const auto &q_rmp : q_motion_policies)
  {
    q_sum = q_sum + *q_rmp;
  }

  /*
   * 4) The combined RMP is itself pulled back into an unconstrained space to
   *    handle joint limits. */

   q_acc = q_sum.f;
}


void MotionPlanner::update(const ros::Time &time, const ros::Duration &period)
{
  double dt = period.toSec();

  q_vel += q_acc * dt;
  q_pos += q_vel * dt;
}


std_msgs::Float64MultiArray MotionPlanner::getPositionCommand()
{
  std_msgs::Float64MultiArray msg;

  for (int i = 0; i < q_pos.size(); i++)
  {
    const double Q_MAX =  3.05;
    const double Q_MIN = -3.05;

    auto sigmoid = [](double u) { return 1.0 / (1.0 + std::exp(-u)); };

    msg.data.push_back((Q_MAX - Q_MIN) * sigmoid(q_pos[i]) + Q_MIN);
  }

  return msg;
}


}  // namespace
