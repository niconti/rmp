#ifndef RMP_MOTION_VISUALIZATION_MARKER_SERVER_H
#define RMP_MOTION_VISUALIZATION_MARKER_SERVER_H
#include <string>
#include <vector>
#include <memory>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
// Eigen
#include <Eigen/Dense>


namespace rmp {


class IMarkerServer {

  // Node
  ros::NodeHandle node;

  // Published Topics
  ros::Publisher target_pose_pub;
  ros::Publisher obstacle_pose_pub;

  geometry_msgs::PoseStamped target;
  geometry_msgs::PoseStamped obstacle;

  geometry_msgs::PoseStamped last_target;
  geometry_msgs::PoseStamped last_obstacle;

public:

  IMarkerServer(const ros::NodeHandle &node = ros::NodeHandle())
  : node(node) 
  { 
    target_pose_pub = this->node.advertise<geometry_msgs::PoseStamped>("target_pose", 10);
    obstacle_pose_pub = this->node.advertise<geometry_msgs::PoseStamped>("obstacle_pose", 10);
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

        double x, y, z;
        x = feedback->pose.position.x;
        y = feedback->pose.position.y;
        z = feedback->pose.position.z;

        double qx, qy, qz, qw;
        qx = feedback->pose.orientation.x;
        qy = feedback->pose.orientation.y;
        qz = feedback->pose.orientation.z;
        qw = feedback->pose.orientation.w;

        if (feedback->marker_name == "goal_marker")
        {
          target.header.frame_id = feedback->header.frame_id;
          target.pose.position.x = x;
          target.pose.position.y = y;
          target.pose.position.z = z;
          target.pose.orientation.x = qx;
          target.pose.orientation.y = qy;
          target.pose.orientation.z = qz;
          target.pose.orientation.w = qw;
        }
        if (feedback->marker_name == "obstacle_marker")
        {
          obstacle.header.frame_id = feedback->header.frame_id;
          obstacle.pose.position.x = x;
          obstacle.pose.position.y = y;
          obstacle.pose.position.z = z;
          obstacle.pose.orientation.x = qx;
          obstacle.pose.orientation.y = qy;
          obstacle.pose.orientation.z = qz;
          obstacle.pose.orientation.w = qw;
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


  void publish()
  {
    if (target != last_target)
    {
      target_pose_pub.publish(target);
      last_target = target;
    }
    if (obstacle != last_obstacle)
    {
      obstacle_pose_pub.publish(obstacle);
      last_obstacle = obstacle;
    }
  }

};

} // namespace
#endif
