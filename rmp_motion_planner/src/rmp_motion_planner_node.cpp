#include <sstream>
#include <string>
#include <vector>
#include <memory>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <interactive_markers/interactive_marker_server.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
// Eigen
#include <Eigen/Dense>
// Boost
#include <boost/bind.hpp>
#include <boost/function.hpp>
//
#include "rmp_motion_planner/interactive_marker.h"
#include "rmp_motion_planner/motion_planner.h"
#include "rmp_motion_planner/motion_policies/redundancy_policy.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rmp_motion_planner_node");

  // Node
  ros::NodeHandle node("~");

  // Parameters
  auto eef_frame = node.param<std::string>("eef_frame", "tool0");
  auto planning_group_name = node.param<std::string>("planning_group", "manipulator");


  double loop_hz;
  if (!node.getParam("hardware_interface/loop_hz", loop_hz))
  {
    std::string param_name = node.resolveName("hardware_interface/loop_hz");
    ROS_FATAL("Failed to retrieve '%s' parameter.", param_name.c_str());
    return 1;
  }

  std::vector<std::string> joints;
  if (!node.getParam("hardware_interface/joints", joints))
  {
    std::string param_name = node.resolveName("hardware_interface/joints");
    ROS_FATAL("Failed to retrieve '%s' parameter.", param_name.c_str());
    return 1;
  }

  /* MOVEIT CPP */
  auto moveit_cpp = std::make_shared<moveit_cpp::MoveItCpp>(node);
  auto planning_component = std::make_shared<moveit_cpp::PlanningComponent>(planning_group_name, moveit_cpp);


  /* RMP PLANNER */
  rmp::MotionPlanner rmp_motion_planner(moveit_cpp->getRobotModel(), planning_component, node);
  if (!rmp_motion_planner.init(joints, eef_frame))
  {
    ROS_FATAL("Failed to initialize RMP Motion Planner");
    return 1;
  }

  {
    Eigen::Vector3d x(0.0, 0.0, 0.0);
    Eigen::Vector3d x_goal(0.0, -0.5, 0.5);

    auto x_rmp = std::make_shared<rmp::TargetPolicy>(x_goal,x);
    rmp_motion_planner.addXMotionPolicy(x_rmp);
  }

  {
    Eigen::Vector3d x(0.0, 0.0, 0.0);
    Eigen::Vector3d x_obs(0.0, -0.5, 0.5);

    auto x_rmp = std::make_shared<rmp::CollisionPolicy>(x_obs,x);
    rmp_motion_planner.addXMotionPolicy(x_rmp);
  }

  // {
  //   Eigen::VectorXd q_goal(6);
  //   q_goal << 1.57, 0.0, -1.57, 0.0, -1.57, 0.0;

  //   auto q_rmp = std::make_shared<rmp::RedundancyPolicy>(q_goal);
  //   rmp_motion_planner.addQMotionPolicy(q_rmp);
  // }

  /* INTERACTIVE MARKER SERVER */
  interactive_markers::InteractiveMarkerServer imarker_server("imarker");

  geometry_msgs::Pose imarker_pose;
  imarker_pose.position.x =  0.0;
  imarker_pose.position.y = -0.5;
  imarker_pose.position.z =  0.5;
  imarker_pose.orientation.x = 0.0;
  imarker_pose.orientation.y = 0.0;
  imarker_pose.orientation.z = 0.0;
  imarker_pose.orientation.w = 1.0;

  {
    auto imarker = rmp::create_goal_imarker("goal_marker", "GOAL", imarker_pose, 0.1);
    imarker_server.insert(imarker, boost::bind(&rmp::MotionPlanner::imarker_feedback_cb, &rmp_motion_planner, _1));
  }

  {
    auto imarker = rmp::create_obstacle_imarker("obstacle_marker", "OBSTACLE", imarker_pose, 0.1);
    imarker_server.insert(imarker, boost::bind(&rmp::MotionPlanner::imarker_feedback_cb, &rmp_motion_planner, _1));
  }

  imarker_server.applyChanges();


  // Published Topics
  auto joint_group_pos_cmd_pub = node.advertise<std_msgs::Float64MultiArray>("/joint_group_controller/command", 1);


  // Loop
  ros::Rate rate(loop_hz);
  ros::Time time_1 = ros::Time::now();
  unsigned long c = 0;
  while (ros::ok())
  {
    rate.sleep();

    ros::spinOnce();

    ros::Time time = ros::Time::now();
    ros::Duration period = time - time_1;

    if (c % 10 == 0)
    {
      rmp_motion_planner.computeMotionPolicies();
    }

    rmp_motion_planner.update(time, period);

    auto msg = rmp_motion_planner.getPositionCommand();

    joint_group_pos_cmd_pub.publish(msg);

    time_1 = time;
    c++;
  }

  return 0;
}
