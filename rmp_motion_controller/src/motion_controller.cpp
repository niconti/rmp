# include "rmp_motion_controller/motion_controller.h"


namespace rmp {


template <class HardwareInterface>
void MotionController<HardwareInterface>::updateState() 
{
  for (int i = 0; i < joint_handles.size(); i++)
  {
    Base::current_jpos(i) = joint_handles[i].getPosition();
    Base::current_jvel(i) = joint_handles[i].getVelocity();
    Base::current_jacc(i) = 0.0;
  }
}

template <class HardwareInterface>
void MotionController<HardwareInterface>::sendCommand() 
{
  for (int i = 0; i < joint_handles.size(); i++)
  {
    joint_handles[i].setCommand(Base::q_pos[i]);
  }
}


template <class HardwareInterface>
bool MotionController<HardwareInterface>::init(HardwareInterface* robot_hw, ros::NodeHandle &node) 
{
  std::string robot_description;
  if (!node.getParam(ROBOT_DESCRIPTION, robot_description))
  {
    std::string param_name = node.resolveName(ROBOT_DESCRIPTION);
    ROS_ERROR_NAMED(LOGNAME, "Failed to retrive '%s' from parameter server.", param_name.c_str());
    return false;
  }

  if (!node.getParam("base_frame", base_frame))
  {
    std::string param_name = node.resolveName("base_frame");
    ROS_ERROR_NAMED(LOGNAME, "Failed to retrive '%s' from parameter server.", param_name.c_str());
    return false;
  }

  if (!node.getParam("eef_frame", eef_frame))
  {
    std::string param_name = node.resolveName("eef_frame");
    ROS_ERROR_NAMED(LOGNAME, "Failed to retrive '%s' from parameter server.", param_name.c_str());
    return false;
  }

  urdf::Model urdf_model;
  if (!urdf_model.initString(robot_description))
  {
    std::string param_name = node.resolveName(ROBOT_DESCRIPTION);
    ROS_ERROR_NAMED(LOGNAME, "Failed to parse URDF Model from '%s'", param_name.c_str());
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(urdf_model, Base::kdl_tree))
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to parse URDF Model");
    return false;
  }

  if (!Base::kdl_tree.getChain(base_frame, eef_frame, Base::kdl_chain))
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to get kinematics chain from '%s' to '%s'", base_frame.c_str(), eef_frame.c_str());
    return false;
  }

  ROS_INFO_NAMED(LOGNAME, "Number of Joints: %d", Base::kdl_chain.getNrOfJoints());
  ROS_INFO_NAMED(LOGNAME, "Number of Segments: %d", Base::kdl_chain.getNrOfSegments());

  if (!node.getParam("joints", joint_names))
  {
    std::string param_name = node.resolveName("joints");
    ROS_ERROR_NAMED(LOGNAME, "Failed to retrive '%s' from parameter server.", param_name.c_str());
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

  target_pose_sub = node.subscribe("target_pose", 3, &MotionController<HardwareInterface>::targetFrameCallback, this);
  obstacle_pose_sub = node.subscribe("obstacle_pose", 3, &MotionController<HardwareInterface>::obstacleFrameCallback, this);

  return true;
}


template <class HardwareInterface>
void MotionController<HardwareInterface>::starting(const ros::Time &time)
{
  for (int i=0; i < Base::kdl_chain.getNrOfSegments(); i++)
  {
    std::string link_name = Base::kdl_chain.getSegment(i).getName();
    std::cout << "Link name: " << link_name << std::endl;
  }
  for (int i=1; i < Base::kdl_chain.getNrOfSegments(); i++)
  {
    Eigen::Vector3d x(0.0, 0.0, 0.0);
    Eigen::Vector3d x_obs(0.5, 0, 0.5);

    auto link_policy = std::make_shared<rmp::LinkPolicy>(Base::kdl_chain,i);
    link_policy->addObstacle(x_obs);
    
    Base::link_policies.push_back(link_policy);
  }

  {
    Eigen::Translation3d o_goal(0.5, 0, 0.5);
    Eigen::Quaterniond R_goal(0, 1, 0, 0);
    Eigen::Isometry3d X_goal = o_goal * R_goal;

    auto eef_policy = std::make_shared<rmp::EndEffectorPolicy>(Base::kdl_chain);
    eef_policy->addTarget(X_goal);    
    
    Eigen::Vector3d x_obs(0.5, 0, 0.5);
    eef_policy->addObstacle(x_obs);

    Base::eef_policy = eef_policy;
  }

  {
    Eigen::VectorXd q_goal(7);
    q_goal << 0.0, -0.8, 0.0, -2.35, 0.0, 1.57, 0.8;

    auto joint_policy = std::make_shared<rmp::JointPolicy>(Base::kdl_chain);
    joint_policy->addTarget(q_goal);

    Base::joint_policy = joint_policy;
  }


  updateState();
  Base::q_pos = Base::current_jpos.data;
  Base::q_vel = Base::current_jvel.data;
  Base::q_acc = Base::current_jacc.data;
  sendCommand();
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
  Eigen::Translation3d o_goal(x, y, z);

  double qx, qy, qz, qw;
  qx = target.pose.orientation.x;
  qy = target.pose.orientation.y;
  qz = target.pose.orientation.z;
  qw = target.pose.orientation.w;
  Eigen::Quaterniond R_goal(qw, qx, qy, qz);

  Eigen::Isometry3d X_goal = o_goal * R_goal;
  Base::eef_policy->setTarget(X_goal);
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

  for (const auto &policy : Base::link_policies)
  {
    policy->setObstacle(x_obs);
  }
}


}