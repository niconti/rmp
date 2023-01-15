#ifndef RMP_MOTION_PLANNER_MOTION_PLANNER_H
#define RMP_MOTION_PLANNER_MOTION_PLANNER_H
#include <cmath>
#include <string>
#include <vector>
#include <memory>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/moveit_cpp/planning_component.h>
// Eigen
#include <Eigen/Dense>
//
#include "rmp_motion_planner/math.h"
#include "rmp_motion_planner/motion_policies.h"
#include "rmp_motion_planner/motion_policies/target_policy.h"
#include "rmp_motion_planner/motion_policies/collision_policy.h"


namespace rmp {


class MotionPlanner {

  ros::NodeHandle node;

  std::string eef_frame;
  std::vector<std::string> joint_names;

  moveit::core::RobotModelConstPtr robot_model;
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_component;

  Eigen::VectorXd q_pos;
  Eigen::VectorXd q_vel;
  Eigen::VectorXd q_acc;

  std::vector<std::shared_ptr<XMotionPolicy>> x_motion_policies;
  std::vector<std::shared_ptr<QMotionPolicy>> q_motion_policies;

  MotionPolicy pullback(const XMotionPolicy &x_rmp, const Eigen::MatrixXd &J);
  MotionPolicy pushforward(const QMotionPolicy &q_rmp, const Eigen::MatrixXd &J);

public:

  MotionPlanner(
    const moveit::core::RobotModelConstPtr &robot_model,
    const std::shared_ptr<moveit_cpp::PlanningComponent> &planning_component,
    const ros::NodeHandle &node = ros::NodeHandle()) :
    node(node),
    robot_model(robot_model),
    planning_component(planning_component) { }


  bool init(const std::vector<std::string> &joint_names, const std::string &eef_frame)
  {
    this->eef_frame = eef_frame;
    this->joint_names = joint_names;

    for (int i = 0; i < joint_names.size(); i++)
    {
      ROS_INFO("%d) Joint Name: %s", i, joint_names[i].c_str());
    }

    const int n_joints = joint_names.size();

    q_pos = Eigen::VectorXd::Zero(n_joints);
    q_vel = Eigen::VectorXd::Zero(n_joints);
    q_acc = Eigen::VectorXd::Zero(n_joints);

    return true;
  }


  void imarker_feedback_cb(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr &feedback)
  {
    switch (feedback->event_type)
    {
      case visualization_msgs::InteractiveMarkerFeedback::KEEP_ALIVE:
        ROS_DEBUG("IMarker '%s' feedback event: %s", feedback->marker_name.c_str(), "KEEP_ALIVE");
        break;
      case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        ROS_DEBUG("IMarker '%s' feedback event: %s", feedback->marker_name.c_str(), "POSE_UPDATE");

        if (feedback->marker_name == "goal_marker")
        {
          double x = feedback->pose.position.x;
          double y = feedback->pose.position.y;
          double z = feedback->pose.position.z;

          Eigen::Vector3d b(0.0, 0.0, 0.0);
          Eigen::Vector3d x_goal(x, y, z);

          auto x_rmp = std::make_shared<rmp::TargetPolicy>(x_goal,b);

          x_motion_policies[0] = x_rmp;
        }
        if (feedback->marker_name == "obstacle_marker")
        {
          double x = feedback->pose.position.x;
          double y = feedback->pose.position.y;
          double z = feedback->pose.position.z;

          Eigen::Vector3d b(0.0, 0.0, 0.0);
          Eigen::Vector3d x_goal(x, y, z);

          auto x_rmp = std::make_shared<rmp::CollisionPolicy>(x_goal,b);

          x_motion_policies[1] = x_rmp;
        }
        break;
      case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        ROS_DEBUG("IMarker '%s' feedback event: %s", feedback->marker_name.c_str(), "MENU_SELECT");
        break;
      case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        ROS_DEBUG("IMarker '%s' feedback event: %s", feedback->marker_name.c_str(), "BUTTON_CLICK");
        break;
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        ROS_DEBUG("IMarker '%s' feedback event: %s", feedback->marker_name.c_str(), "MOUSE_UP");
        break;
      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_DEBUG("IMarker '%s' feedback event: %s", feedback->marker_name.c_str(), "MOUSE_DOWN");
        break;
      default:
        ROS_WARN("IMarker '%s' feedback event: %s", feedback->marker_name.c_str(), "Unknown");
        break;
    }
  }


  void addXMotionPolicy(const std::shared_ptr<rmp::XMotionPolicy> &rmp);
  void addQMotionPolicy(const std::shared_ptr<rmp::QMotionPolicy> &rmp);

  void clearXMotionPolicies();
  void clearQMotionPolicies();

  void computeMotionPolicies();

  void update(const ros::Time &time, const ros::Duration &period);

  std_msgs::Float64MultiArray getPositionCommand();

};

} // namespace
#endif
