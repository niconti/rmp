#ifndef RMP_MOTION_VISUALIZATION_IMARKER_UTILS_H
#define RMP_MOTION_VISUALIZATION_IMARKER_UTILS_H
#include <string>
#include <vector>
#include <memory>
// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>


namespace rmp {


visualization_msgs::InteractiveMarker
create_goal_imarker(const std::string &name, const std::string &description="", const geometry_msgs::Pose &pose=geometry_msgs::Pose(), double scale=1.0)
{
  // SPHERE MARKER
  visualization_msgs::Marker sphere_marker;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.scale.x = scale;
  sphere_marker.scale.y = scale;
  sphere_marker.scale.z = scale;
  sphere_marker.color.r = 0.0;
  sphere_marker.color.g = 1.0;
  sphere_marker.color.b = 1.0;
  sphere_marker.color.a = 0.5;

  // INTERACTIVE MARKER
  visualization_msgs::InteractiveMarker imarker;
  imarker.header.frame_id = "panda_link0";
  imarker.header.stamp = ros::Time(0);
  imarker.name = name;
  imarker.description = description;
  imarker.pose = pose;
  imarker.scale = scale;

  // MOVE X
  visualization_msgs::InteractiveMarkerControl move_x_control;
  move_x_control.name = "move_x";
  move_x_control.orientation.x = 0.0;
  move_x_control.orientation.y = 0.0;
  move_x_control.orientation.z = 0.0;
  move_x_control.orientation.w = 1.0;
  move_x_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  imarker.controls.push_back(move_x_control);

  // MOVE Y
  visualization_msgs::InteractiveMarkerControl move_y_control;
  move_y_control.name = "move_y";
  move_y_control.orientation.x = 0.0;
  move_y_control.orientation.y = 0.0;
  move_y_control.orientation.z = 0.7071068;
  move_y_control.orientation.w = 0.7071068;
  move_y_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  imarker.controls.push_back(move_y_control);

  // MOVE Z
  visualization_msgs::InteractiveMarkerControl move_z_control;
  move_z_control.name = "move_z";
  move_z_control.orientation.x =  0.0;
  move_z_control.orientation.y = -0.7071068;
  move_z_control.orientation.z =  0.0;
  move_z_control.orientation.w =  0.7071068;
  move_z_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  imarker.controls.push_back(move_z_control);

  // ROTATE X
  visualization_msgs::InteractiveMarkerControl rotate_x_control;
  rotate_x_control.name = "rotate_x";
  rotate_x_control.orientation.x = 0.0;
  rotate_x_control.orientation.y = 0.0;
  rotate_x_control.orientation.z = 0.0;
  rotate_x_control.orientation.w = 1.0;
  rotate_x_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  imarker.controls.push_back(rotate_x_control);

  // ROTATE Y
  visualization_msgs::InteractiveMarkerControl rotate_y_control;
  rotate_y_control.name = "rotate_y";
  rotate_y_control.orientation.x = 0.0;
  rotate_y_control.orientation.y = 0.0;
  rotate_y_control.orientation.z = 0.7071068;
  rotate_y_control.orientation.w = 0.7071068;
  rotate_y_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  imarker.controls.push_back(rotate_y_control);

  // ROTATE Z
  visualization_msgs::InteractiveMarkerControl rotate_z_control;
  rotate_z_control.name = "rotate_z";
  rotate_z_control.orientation.x =  0.0;
  rotate_z_control.orientation.y = -0.7071068;
  rotate_z_control.orientation.z =  0.0;
  rotate_z_control.orientation.w =  0.7071068;
  rotate_z_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  imarker.controls.push_back(rotate_z_control);

  // FULL 3D
  visualization_msgs::InteractiveMarkerControl sphere_control;
  sphere_control.name = "full_3d";
  sphere_control.markers.push_back(sphere_marker);
  sphere_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  sphere_control.always_visible = true;
  imarker.controls.push_back(sphere_control);

  return imarker;
}


visualization_msgs::InteractiveMarker
create_obstacle_imarker(const std::string &name, const std::string &description="", const geometry_msgs::Pose &pose=geometry_msgs::Pose(), double scale=1.0)
{
  // SPHERE MARKER
  visualization_msgs::Marker sphere_marker;
  sphere_marker.type = visualization_msgs::Marker::SPHERE;
  sphere_marker.scale.x = scale;
  sphere_marker.scale.y = scale;
  sphere_marker.scale.z = scale;
  sphere_marker.color.r = 1.0;
  sphere_marker.color.g = 0.0;
  sphere_marker.color.b = 0.0;
  sphere_marker.color.a = 0.5;

  // INTERACTIVE MARKER
  visualization_msgs::InteractiveMarker imarker;
  imarker.header.frame_id = "panda_link0";
  imarker.header.stamp = ros::Time(0);
  imarker.name = name;
  imarker.description = description;
  imarker.pose = pose;
  imarker.scale = scale;

  // MOVE X
  visualization_msgs::InteractiveMarkerControl move_x_control;
  move_x_control.name = "move_x";
  move_x_control.orientation.x = 0.0;
  move_x_control.orientation.y = 0.0;
  move_x_control.orientation.z = 0.0;
  move_x_control.orientation.w = 1.0;
  move_x_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  imarker.controls.push_back(move_x_control);

  // MOVE Y
  visualization_msgs::InteractiveMarkerControl move_y_control;
  move_y_control.name = "move_y";
  move_y_control.orientation.x = 0.0;
  move_y_control.orientation.y = 0.0;
  move_y_control.orientation.z = 0.7071068;
  move_y_control.orientation.w = 0.7071068;
  move_y_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  imarker.controls.push_back(move_y_control);

  // MOVE Z
  visualization_msgs::InteractiveMarkerControl move_z_control;
  move_z_control.name = "move_z";
  move_z_control.orientation.x =  0.0;
  move_z_control.orientation.y = -0.7071068;
  move_z_control.orientation.z =  0.0;
  move_z_control.orientation.w =  0.7071068;
  move_z_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  imarker.controls.push_back(move_z_control);

  // ROTATE X
  visualization_msgs::InteractiveMarkerControl rotate_x_control;
  rotate_x_control.name = "rotate_x";
  rotate_x_control.orientation.x = 0.0;
  rotate_x_control.orientation.y = 0.0;
  rotate_x_control.orientation.z = 0.0;
  rotate_x_control.orientation.w = 1.0;
  rotate_x_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  imarker.controls.push_back(rotate_x_control);

  // ROTATE Y
  visualization_msgs::InteractiveMarkerControl rotate_y_control;
  rotate_y_control.name = "rotate_y";
  rotate_y_control.orientation.x = 0.0;
  rotate_y_control.orientation.y = 0.0;
  rotate_y_control.orientation.z = 0.7071068;
  rotate_y_control.orientation.w = 0.7071068;
  rotate_y_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  imarker.controls.push_back(rotate_y_control);

  // ROTATE Z
  visualization_msgs::InteractiveMarkerControl rotate_z_control;
  rotate_z_control.name = "rotate_z";
  rotate_z_control.orientation.x =  0.0;
  rotate_z_control.orientation.y = -0.7071068;
  rotate_z_control.orientation.z =  0.0;
  rotate_z_control.orientation.w =  0.7071068;
  rotate_z_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  imarker.controls.push_back(rotate_z_control);

  // FULL 3D
  visualization_msgs::InteractiveMarkerControl sphere_control;
  sphere_control.name = "full_3d";
  sphere_control.markers.push_back(sphere_marker);
  sphere_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  sphere_control.always_visible = true;
  imarker.controls.push_back(sphere_control);

  return imarker;
}


}  // namespace
#endif
