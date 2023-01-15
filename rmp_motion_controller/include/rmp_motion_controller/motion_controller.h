#ifndef RMP_MOTION_CONTROLLER_MOTION_CONTROLLER_H
#define RMP_MOTION_CONTROLLER_MOTION_CONTROLLER_H
#include <string>
#include <vector>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
//
#include "rmp_motion_controller/motion_controller_base.h"


namespace rmp  {


template <class HardwareInterface>
class MotionController : public MotionControllerBase<HardwareInterface> {

  typedef MotionControllerBase<HardwareInterface> Base;

  const std::string ROBOT_DESCRIPTION = "/robot_description";

  std::vector<std::string> joint_names;
  std::string base_frame;
  std::string eef_frame;

  // Node
  ros::NodeHandle node;

  // Hardware Interface
  std::vector<hardware_interface::JointHandle> joint_handles;

  void updateState();

  void sendCommand();

public:

  bool init(HardwareInterface* robot_hw, ros::NodeHandle &node);

  void starting(const ros::Time &time);
  
  void update(const ros::Time &time, const ros::Duration &period);

};

}

namespace position_controllers
{
  /**
   * @brief Cartesian motion controller that transforms end-effector target motion into commands for a chain of position interfaces.
   */
  typedef rmp::MotionController<hardware_interface::PositionJointInterface> RiemmanianMotionController;
}

namespace velocity_controllers
{
  /**
   * @brief Cartesian motion controller that transforms end-effector target motion into commands for a chain of velocity interfaces.
   */
  typedef rmp::MotionController<hardware_interface::VelocityJointInterface> RiemmanianMotionController;
}

PLUGINLIB_EXPORT_CLASS(position_controllers::RiemmanianMotionController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(velocity_controllers::RiemmanianMotionController, controller_interface::ControllerBase)
#endif