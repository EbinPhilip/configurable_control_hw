#ifndef __HARDWARE_INTERFACE_ACCESSORS_H__
#define __HARDWARE_INTERFACE_ACCESSORS_H__

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <transmission_interface/transmission_interface.h>

// interface for accessing joint resource interfaces  on robot hw
struct Joint_Interfaces_Accessor
{
    virtual hardware_interface::JointStateInterface &getJointStateInterface() = 0;
    virtual hardware_interface::PositionJointInterface &getPositionJointInterface() = 0;
    virtual hardware_interface::PosVelJointInterface &getPosVelJointInterface() = 0;
};

// interface for accessing transmission resource interface resources on robot hw
struct Transmission_Interfaces_Accessor
{
    virtual transmission_interface::ActuatorToJointStateInterface &getActuatorToJointStateInterface() = 0;
    virtual transmission_interface::JointToActuatorStateInterface &getJointToActuatorStateInterface() = 0;
};

#endif
