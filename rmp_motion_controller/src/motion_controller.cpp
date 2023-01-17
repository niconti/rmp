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


  target_pose_sub = node.subscribe("target_pose", 3, &MotionController<HardwareInterface>::targetFrameCallback, this);
  obstacle_pose_sub = node.subscribe("obstacle_pose", 3, &MotionController<HardwareInterface>::obstacleFrameCallback, this);

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

  {
    Eigen::Vector3d x(0.0, 0.0, 0.0);
    Eigen::Vector3d x_obs(0.5, 0, 0.5);

    auto x_rmp = std::make_shared<rmp::CollisionPolicy>(x_obs,x);
    Base::addXMotionPolicy(x_rmp);
  }

  {
    Eigen::VectorXd q_goal(7);
    q_goal << 0.0, -0.8, 0.0, -2.35, 0.0, 1.57, 0.8;

    auto q_rmp = std::make_shared<rmp::RedundancyPolicy>(q_goal);
    Base::addQMotionPolicy(q_rmp);
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


template <class HardwareInterface>
void MotionController<HardwareInterface>::targetFrameCallback(const geometry_msgs::PoseStamped &target)
{
  if (target.header.frame_id != base_frame)
  {
    std::string frame_id = target.header.frame_id;
    ROS_WARN_THROTTLE(3, "Target frame_id: '%s' doesn't match base_frame: '%s'. Skip.", frame_id.c_str(), base_frame.c_str());
    return;
  }

  double x, y, z;
  x = target.pose.position.x;
  y = target.pose.position.y;
  z = target.pose.position.z;

  Eigen::Vector3d b(0.0, 0.0, 0.0);
  Eigen::Vector3d x_goal(x, y, z);

  // auto x_rmp = std::make_shared<rmp::TargetPolicy>(x_goal,b);

  auto x_rmp = Base::x_motion_policies[0];

  std::dynamic_pointer_cast<rmp::TargetPolicy>(x_rmp)->setGoal(x_goal);
}

template <class HardwareInterface>
void MotionController<HardwareInterface>::obstacleFrameCallback(const geometry_msgs::PoseStamped &obstacle)
{
  if (obstacle.header.frame_id != base_frame)
  {
    std::string frame_id = obstacle.header.frame_id;
    ROS_WARN_THROTTLE(3, "Obstacle frame_id: '%s' doesn't match base_frame: '%s'. Skip.", frame_id.c_str(), base_frame.c_str());
    return;
  }

  double x, y, z;
  x = obstacle.pose.position.x;
  y = obstacle.pose.position.y;
  z = obstacle.pose.position.z;

  Eigen::Vector3d b(0.0, 0.0, 0.0);
  Eigen::Vector3d x_obs(x, y, z);

  // auto x_rmp = std::make_shared<rmp::CollisionPolicy>(x_obs,b);

  auto x_rmp = Base::x_motion_policies[1];

  std::dynamic_pointer_cast<rmp::CollisionPolicy>(x_rmp)->setObstacle(x_obs);
}


}