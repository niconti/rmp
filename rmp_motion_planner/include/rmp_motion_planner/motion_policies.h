#ifndef RMP_MOTION_PLANNER_MOTION_POLICIES_H
#define RMP_MOTION_PLANNER_MOTION_POLICIES_H
#include "rmp_motion_planner/math.h"
// Eigen
#include <Eigen/Dense>


namespace rmp {


class MotionPolicy {
public:

  Eigen::VectorXd f;
  Eigen::MatrixXd A;

  MotionPolicy operator+(const MotionPolicy &other);

};


class XMotionPolicy : public MotionPolicy {
protected:

  virtual Eigen::Vector3d policy(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) = 0;
  virtual Eigen::Matrix3d metric(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) = 0;

public:

  Eigen::Vector3d x;

  void update(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel)
  {
    f = policy(x_pos,x_vel);
    A = metric(x_pos,x_vel);
  }

};


class QMotionPolicy : public MotionPolicy {
protected:

  virtual Eigen::VectorXd policy(const Eigen::VectorXd &q_pos, const Eigen::VectorXd &q_vel) = 0;
  virtual Eigen::MatrixXd metric(const Eigen::VectorXd &q_pos, const Eigen::VectorXd &q_vel) = 0;

public:

  Eigen::VectorXd q;

  void update(const Eigen::VectorXd &q_pos, const Eigen::VectorXd &q_vel)
  {
    f = policy(q_pos,q_vel);
    A = metric(q_pos,q_vel);
  }

};

}  // namespace
#endif
