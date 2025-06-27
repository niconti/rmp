#ifndef RMP_MOTION_CONTROLLER_MOTION_POLICIES_H
#define RMP_MOTION_CONTROLLER_MOTION_POLICIES_H
#include "rmp_motion_controller/math.h"
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

  virtual Eigen::Vector3d policy(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel){};
  virtual Eigen::Matrix3d metric(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel){};

public:

  Eigen::Vector3d x;

  void update(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel)
  {
    f = policy(x_pos,x_vel);
    A = metric(x_pos,x_vel);
  }

  XMotionPolicy operator+(const XMotionPolicy &other)
  {
    Eigen::VectorXd f1 = f;
    Eigen::MatrixXd A1 = A;

    Eigen::VectorXd f2 = other.f;
    Eigen::MatrixXd A2 = other.A;

    XMotionPolicy rmp;
    rmp.f = pinv(A1 + A2) * (A1*f1 + A2*f2);
    rmp.A = A1 + A2;

    return rmp;
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


class JMotionPolicy : public MotionPolicy {
protected:

  virtual Eigen::VectorXd policy(const double &q_pos, const double &q_vel) = 0;
  virtual Eigen::MatrixXd metric(const double &q_pos, const double &q_vel) = 0;

public:

  double x;

  void update(const double &q_pos, const double &q_vel)
  {
    f = policy(q_pos,q_vel);
    A = metric(q_pos,q_vel);
  }

};


}  // namespace
#endif
