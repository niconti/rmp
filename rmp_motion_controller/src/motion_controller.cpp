# include "rmp_motion_controller/motion_controller.h"


namespace rmp {


template <class HardwareInterface>
void MotionController<HardwareInterface>::updateState() {

  for (int i = 0; i < joint_handles.size(); i++)
  {
    Base::current_jpos(i) = joint_handles[i].getPosition();
    Base::current_jvel(i) = joint_handles[i].getVelocity();
    Base::current_jacc(i) = 0.0;
  }
}

template <class HardwareInterface>
void MotionController<HardwareInterface>::sendCommand() {

  for (int i = 0; i < joint_handles.size(); i++)
  {
    joint_handles[i].setCommand(Base::q_pos[i]);
  }
}


template <class HardwareInterface>
bool MotionController<HardwareInterface>::init(HardwareInterface* robot_hw, ros::NodeHandle &node) {

  std::string robot_description;
  if (!node.getParam(ROBOT_DESCRIPTION, robot_description))
  {
    std::string param_name = node.resolveName(ROBOT_DESCRIPTION);
    ROS_ERROR("Failed to retrive '%s' from parameter server.", param_name.c_str());
    return false;
  }

  if (!node.getParam("base_frame", base_frame))
  {
    std::string param_name = node.resolveName("base_frame");
    ROS_ERROR("Failed to retrive '%s' from parameter server.", param_name.c_str());
    return false;
  }

  if (!node.getParam("eef_frame", eef_frame))
  {
    std::string param_name = node.resolveName("eef_frame");
    ROS_ERROR("Failed to retrive '%s' from parameter server.", param_name.c_str());
    return false;
  }

  urdf::Model urdf_model;
  if (!urdf_model.initString(robot_description))
  {
    std::string param_name = node.resolveName(ROBOT_DESCRIPTION);
    ROS_ERROR("Failed to parse urdf model from '%s'", param_name.c_str());
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(urdf_model, Base::kdl_tree))
  {
    ROS_ERROR("Failed to parse URDL Model");
    return false;
  }

  if (!Base::kdl_tree.getChain(base_frame, eef_frame, Base::kdl_chain))
  {
    ROS_ERROR("Failed to get kinematics chain from '%s' to '%s'", base_frame.c_str(), eef_frame.c_str());
    return false;
  }


  if (!node.getParam("joints", joint_names))
  {
    std::string param_name = node.resolveName("joints");
    ROS_ERROR("Failed to retrive '%s' from parameter server.", param_name.c_str());
    return false;
  }

  const int n_joints = joint_names.size();
  Base::q_pos = Eigen::VectorXd::Zero(n_joints);
  Base::q_vel = Eigen::VectorXd::Zero(n_joints);
  Base::q_acc = Eigen::VectorXd::Zero(n_joints);

  for (int i = 0; i < joint_names.size(); i++)
  {
    try
    {
      joint_handles.push_back(robot_hw->getHandle(joint_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
  }

  Base::current_jpos.resize(n_joints);
  Base::current_jvel.resize(n_joints);
  Base::current_jacc.resize(n_joints);

  Base::fk_pos_solver.reset(new KDL::ChainFkSolverPos_recursive(Base::kdl_chain));
  Base::fk_vel_solver.reset(new KDL::ChainFkSolverVel_recursive(Base::kdl_chain));

  Base::jacobian.resize(n_joints);
  Base::jacobian_solver.reset(new KDL::ChainJntToJacSolver(Base::kdl_chain));

  return true;
}


template <class HardwareInterface>
void MotionController<HardwareInterface>::starting(const ros::Time &time)
{
  {
    Eigen::Vector3d x(0.0, 0.0, 0.0);
    Eigen::Vector3d x_goal(0.5, 0, 0.5);

    auto x_rmp = std::make_shared<rmp::TargetPolicy>(x_goal,x);
    Base::addXMotionPolicy(x_rmp);
  }
}


template <class HardwareInterface>
void MotionController<HardwareInterface>::update(const ros::Time &time, const ros::Duration &period)
{
  updateState();

  Base::computeMotionPolicies();

  double dt = period.toSec();

  Base::q_vel += Base::q_acc * dt;
  Base::q_pos += Base::q_vel * dt;

  sendCommand();
}


}