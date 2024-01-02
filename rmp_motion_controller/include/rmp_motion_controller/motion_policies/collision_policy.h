#ifndef RMP_MOTION_CONTROLLER_MOTION_POLICIES_COLLISION_POLICY_H
#define RMP_MOTION_CONTROLLER_MOTION_POLICIES_COLLISION_POLICY_H
#include "rmp_motion_controller/motion_policies.h"
// STL
#include <cmath>
#include <functional>
// Eigen
#include <Eigen/Dense>


namespace rmp {


class CollisionPolicy : public XMotionPolicy {
private:

  Eigen::Vector3d x_obs;

  double kd = 50.0;            // Damping gain [s^-1]
  double ld = 0.04;            // Length scale controlling increase in acceleration as obstacle is approached
  double ed = 1e-2;            // Offset determining x value at which acceleration diverges (before clipping)
  double vd = 0.01;            // Scale determining velocity dependence of “velocity gating” function
  double kp = 800.;            // Gain for position repulsion term [m/s^2]
  double lp = 0.01;            // Length scale controlling distance dependence of repulsion
  double r  = 0.5;             // Length scale determining distance from obstacle at which RMP is disabled completely
  double nu = 10000;           // Overall priority weight relative to other RMPs
  double lm = 0.02;            // Length scale controlling increase in metric as obstacle is approached
  double em = 0.001;           // Offset determining x value at which metric diverges (before clipping)

  std::function<double(double)> g;


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
    double x = dist(x_pos,x_obs);
    auto n = v_norm(x_pos,x_obs);
    double vel = n.transpose() * x_vel;

    return kp * std::exp(-x/lp) * n - kd * (1.0 - 1.0 / (1.0 + std::exp(-vel/vd))) * vel / ((x/ld) + ed) * n;
  }


  Eigen::Matrix3d metric(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) override
  {
    double x = dist(x_pos,x_obs);
    auto n = v_norm(x_pos,x_obs);
    double vel = n.transpose() * x_vel;

    return (1.0 - 1.0 / (1.0 + std::exp(-vel/vd))) * g(x) * nu / ((x/lm) + em) * n * n.transpose();
  }

public:

  CollisionPolicy(const Eigen::VectorXd &x_obs, const Eigen::Vector3d &x=Eigen::Vector3d::Zero())
  {
    this->x = x;
    this->x_obs = x_obs;

    g = [=](double value)
    {
      if (value < r)
      {
        double c2, c1, c0;
        c2 =  1.0 / (r * r);
        c1 = -2.0 / r;
        c0 =  1.0;
        return c2 * value * value + c1 * value + c0;
      }
      else
      {
        return 0.0;
      }
    };

  }


  void setObstacle(const Eigen::Vector3d &x_obs)
  {
    this->x_obs = x_obs;
  }

};

}  // namespace
#endif
