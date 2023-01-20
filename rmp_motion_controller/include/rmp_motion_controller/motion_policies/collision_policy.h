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

  std::function<double(double)> w;
  std::function<double(double)> u;
  std::function<double(double)> dw;
  std::function<double(double)> du;


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

  Eigen::Vector3d soft_norm(const Eigen::Vector3d &v)
  {
    double c = 100.0;

    auto h = [&](double z) { return z + (1.0 / c) * std::log(1 + std::exp(-2.0 * c * z)); };

    return v / h(v.norm());
  }


  Eigen::Vector3d policy(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) override
  {
    // return alpha(dist(x_pos,x_obs)) * v_norm(x_pos,x_obs) - beta(dist(x_pos,x_obs)) * (v_norm(x_pos,x_obs) * v_norm(x_pos,x_obs).transpose()) * x_vel;

    auto n = v_norm(x_pos,x_obs);

    double s_pos = dist(x_pos,x_obs);
    double s_vel = n.transpose() * x_vel;

    double alpha = 10e-6;
    return (-alpha * w(s_pos) * dw(s_pos) - 0.5 * std::pow(s_vel,2) * u(s_vel) * dw(s_pos)) * v_norm(x_pos,x_obs);
  }


  Eigen::Matrix3d metric(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) override
  {
    Eigen::Vector3d x_acc = policy(x_pos,x_vel);
    
    auto n = v_norm(x_pos,x_obs);
    
    double s_pos = dist(x_pos,x_obs);
    double s_vel = n.transpose() * x_vel;

    return (w(s_pos) * u(s_vel) + 0.5 * s_vel * w(s_pos) * du(s_vel)) * soft_norm(x_acc) * soft_norm(x_acc).transpose();
  }

public:

  CollisionPolicy(const Eigen::VectorXd &x_obs, const Eigen::Vector3d &x=Eigen::Vector3d::Zero())
  {
    this->x = x;
    this->x_obs = x_obs;

    w = [](double value)
    {
      // double r = 1.0;
      // double v = std::fmax(0, r - value);
      // return v * v / value;
      return 1.0 / std::pow(value,4);
    };

    dw = [](double value)
    {
      return -4.0 / std::pow(value,5);
    };

    u = [](double value)
    {
      // if (value < 0)
      // {
      //   double sigma = 1.0;
      //   return 1 - std::exp(-(value * value) / 2.0 * std::pow(sigma,2));
      // }
      // else
      // {
      //   return 0.;
      // }
      return 10e-9 + std::fmin(0, value) * value;
    };

    du = [](double value)
    {
      // if (value < 0)
      // {
      //   double sigma = 1.0;
      //   return -(value / std::pow(sigma,2)) * std::exp(-(value * value) / 2.0 * std::pow(sigma,2));
      // }
      // else
      // {
      //   return 0.;
      // }
      return 2.0 * std::fmin(0, value);
    };

    // auto g = [&w,&u](double pos, double vel)
    // {
    //   return w(pos) * u(vel);
    // };

    // auto m = [&w,&u,&du](double pos, double vel)
    // {
    //   return w(pos) * u(vel) + 0.5 * w(pos) * vel * du(vel);
    // };

  }


  void setObstacle(const Eigen::Vector3d &x_obs)
  {
    this->x_obs = x_obs;
  }

};

}  // namespace
#endif
