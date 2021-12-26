#ifndef __JOINT_PROPERTIES_H__
#define __JOINT_PROPERTIES_H__

#include "Control_Properties.h"

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/posvel_command_interface.h>

#include <rotational_units/Rotational_Units.h>

#include <string>
#include <memory>

struct Joint_Interfaces_Accessor;

struct Joint_Properties : public Controllable_Entity
{
    Joint_Properties(const std::string &name) : joint_name(name),
                                                joint_state_(joint_name, &state.position, &state.velocity, &state.effort)
    {
    }

    virtual void registerJointInterfaces(Joint_Interfaces_Accessor &);

    virtual void setDefaultVelocity() = 0;

    std::string joint_name;

protected:
    hardware_interface::JointStateHandle joint_state_;
};

struct Position_Joint_Properties : public Joint_Properties
{
    Position_Joint_Properties(const std::string &name, RUnits::Radians_Per_Sec default_velocity = 3.141593)
        : Joint_Properties(name),
          joint_position_(joint_state_, &command.position),
          default_velocity_(default_velocity)
    {
    }

    virtual void registerJointInterfaces(Joint_Interfaces_Accessor &) override;

    virtual void setDefaultVelocity() override;

protected:
    hardware_interface::JointHandle joint_position_;
    RUnits::Radians_Per_Sec default_velocity_;
};

struct PosVel_Joint_Properties : public Joint_Properties
{
    PosVel_Joint_Properties(const std::string &name) : Joint_Properties(name),
                                                       pos_vel_handle_(joint_state_, &command.position, &command.velocity)
    {
    }

    virtual void registerJointInterfaces(Joint_Interfaces_Accessor &) override;

    virtual void setDefaultVelocity() override;

protected:
    hardware_interface::PosVelJointHandle pos_vel_handle_;
};

typedef std::shared_ptr<Joint_Properties> Joint_Properties_Ptr;
typedef std::shared_ptr<std::map<std::string, Joint_Properties_Ptr>> Joint_Map;

#endif