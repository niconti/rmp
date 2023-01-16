#include <string>
#include <vector>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/Pose.h>
// Boost
#include <boost/bind.hpp>
#include <boost/function.hpp>
//
#include "rmp_motion_visualization/imarker_server.h"
#include "rmp_motion_visualization/imarker_utils.h"


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rmp_motion_visualization_node");

  // Node
  ros::NodeHandle node("~");

  // Parameters
  auto freq = node.param<double>("frequency", 10.0);


  /* INTERACTIVE MARKER SERVER */
  interactive_markers::InteractiveMarkerServer imarker_server("imarker");

  geometry_msgs::Pose imarker_pose;
  imarker_pose.position.x = 0.5;
  imarker_pose.position.y = 0.0;
  imarker_pose.position.z = 0.5;
  imarker_pose.orientation.x = 0.0;
  imarker_pose.orientation.y = 0.0;
  imarker_pose.orientation.z = 0.0;
  imarker_pose.orientation.w = 1.0;

  rmp::IMarkerServer rmp_imarker_server(node);

  {
    auto imarker = rmp::create_goal_imarker("goal_marker", "GOAL", imarker_pose, 0.1);
    imarker_server.insert(imarker, boost::bind(&rmp::IMarkerServer::imarker_feedback_cb, &rmp_imarker_server, _1));
  }

  {
    auto imarker = rmp::create_obstacle_imarker("obstacle_marker", "OBSTACLE", imarker_pose, 0.1);
    imarker_server.insert(imarker, boost::bind(&rmp::IMarkerServer::imarker_feedback_cb, &rmp_imarker_server, _1));
  }

  imarker_server.applyChanges();


  // Loop
  ros::Rate rate(freq);
  while (ros::ok())
  {
    ros::spinOnce();

    rmp_imarker_server.publish();
    
    rate.sleep();
  }

  return 0;
}
