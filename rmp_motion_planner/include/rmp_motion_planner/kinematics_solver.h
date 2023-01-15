#ifndef RIEMMANIAN_MOTION_POLICIES_KINEMATIC_SOLVER_H
#define RIEMMANIAN_MOTION_POLICIES_KINEMATIC_SOLVER_H
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <XmlRpcValue.h>
#include <XmlRpcException.h>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>


class KinematicsSolver {

  const std::string LOGNAME = "KinematicSolver";

  struct DHParameter
  {
    double d;
    double theta;
    double a;
    double alpha;
  };

  std::vector<DHParameter> dh;

public:

  KinematicsSolver() { }


  bool init(const XmlRpc::XmlRpcValue &dh_param)
  {
    for (int i = 0; i < dh_param.size(); i++)
    {
      try
      {
        DHParameter dh_parameter;
        dh_parameter.d     = dh_param[i]["d"];
        dh_parameter.theta = dh_param[i]["theta"];
        dh_parameter.a     = dh_param[i]["a"];
        dh_parameter.alpha = dh_param[i]["alpha"];
        dh.push_back(dh_parameter);
      }
      catch (const XmlRpc::XmlRpcException &ex)
      {
        ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to parse DH parameter: " << ex.getMessage());
        return false;
      }
    }

    return true;
  }


  bool forward_ik(Eigen::Isometry3d &x, const Eigen::VectorXd &q);
  
  bool jacobian(Eigen::MatrixXd &J, const Eigen::VectorXd &q);

};
#endif
