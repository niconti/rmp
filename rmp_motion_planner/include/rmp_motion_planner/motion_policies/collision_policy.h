#ifndef RMP_MOTION_PLANNER_MOTION_POLICIES_COLLISION_POLICY_H
#define RMP_MOTION_PLANNER_MOTION_POLICIES_COLLISION_POLICY_H
#include "rmp_motion_planner/motion_policies.h"
// STL
#include <cmath>
#include <functional>
// Eigen
#include <Eigen/Dense>


namespace rmp {


class CollisionPolicy : public XMotionPolicy {
private:

  Eigen::Vector3d x_obs;

  std::function<double(double)> alpha;
  std::function<double(double)> beta;
  std::function<double(double)> weight;


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
    double c = 10.0;

    auto h = [&](double z) { return z + (1.0 / c) * std::log(1 + std::exp(-2.0 * c * z)); };

    return v / h(v.norm());
  }


  Eigen::Vector3d policy(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) override
  {
    std::cout << "obstacle dist: " << dist(x_pos,x_obs) << std::endl;
    std::cout << "obstacle alpha: " << alpha(dist(x_pos,x_obs)) << std::endl;
    return alpha(dist(x_pos,x_obs)) * v_norm(x_pos,x_obs) - beta(dist(x_pos,x_obs)) * (v_norm(x_pos,x_obs) * v_norm(x_pos,x_obs).transpose()) * x_vel;
  }


  Eigen::Matrix3d metric(const Eigen::Vector3d &x_pos, const Eigen::Vector3d &x_vel) override
  {
    Eigen::Vector3d x_acc = policy(x_pos,x_vel);
    return weight(dist(x_pos,x_obs)) * soft_norm(x_acc) * soft_norm(x_acc).transpose();
  }

public:

  CollisionPolicy(const Eigen::VectorXd &x_obs, const Eigen::Vector3d &x=Eigen::Vector3d::Zero())
  {
    this->x = x;
    this->x_obs = x_obs;

    alpha = [](double value)
    {
      double n = 10.0;
      double v = 0.1;
      return n * std::exp(-value / v);
    };

    beta = [](double value)
    {
      double n = 10.0;
      double v = 0.1;
      return n / (value / v);
    };

    weight = [](double value)
    {
      double r  = 1.0;
      double c2 = 1.0 / (r * r);
      double c1 = -2.0 / r;
      double c0 = 1.0;
      return c2 * value * value + c1 * value + c0;
    };
  }


  void setObstacle(const Eigen::Vector3d &x_obs)
  {
    this->x_obs = x_obs;
  }

};

}  // namespace
#endif
