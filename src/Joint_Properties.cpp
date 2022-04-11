#include "Joint_Properties.h"
#include "Hardware_Interface_Accessors.h"

using namespace hardware_interface;

void Joint_Properties::registerJointInterfaces(Joint_Interfaces_Accessor& accessor)
{
    JointStateInterface& joint_interface = accessor.getJointStateInterface();
    joint_interface.registerHandle(joint_state_);
}

void Position_Joint_Properties::registerJointInterfaces(Joint_Interfaces_Accessor& accessor)
{
    PositionJointInterface& joint_interface = accessor.getPositionJointInterface();
    joint_interface.registerHandle(joint_position_);
    Joint_Properties::registerJointInterfaces(accessor);
}

void Position_Joint_Properties::setDefaultVelocity()
{
    command.velocity = default_velocity_;
}

void PosVel_Joint_Properties::registerJointInterfaces(Joint_Interfaces_Accessor& accessor)
{
    PosVelJointInterface& joint_interface = accessor.getPosVelJointInterface();
    joint_interface.registerHandle(pos_vel_handle_);
    Joint_Properties::registerJointInterfaces(accessor);
}

void PosVel_Joint_Properties::setDefaultVelocity()
{
    
}