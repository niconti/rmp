#include "rmp_motion_controller/motion_controller_base.h"


namespace rmp {


template class MotionControllerBase<hardware_interface::PositionJointInterface>;
template class MotionControllerBase<hardware_interface::VelocityJointInterface>;
}