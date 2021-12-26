#include "Transmission_Properties.h"
#include "Hardware_Interface_Accessors.h"
#include "Actuator_Control_Interface.h"

#include <stdexcept>

using namespace transmission_interface;

Transmission_Properties::Transmission_Properties(const std::string &name, Transmission_Ptr transmission)
    : transmission_name(name),
      transmission_type(transmission)
{
}

void Transmission_Properties::registerTransmission(Transmission_Interfaces_Accessor &accessor,
                                                   Actuator_Control_Interface &actuator_interface,
                                                   Joint_Map joint_map)
{
    for (int i = 0; i < joints.size(); ++i)
    {
        auto joint_it = joint_map->find(joints[i]);
        if (joint_it == joint_map->end())
        {
            throw std::runtime_error("");
        }
        joint_state_data_.position.push_back(&(joint_it->second->state.position));
        joint_state_data_.velocity.push_back(&(joint_it->second->state.velocity));
        joint_state_data_.effort.push_back(&(joint_it->second->state.effort));

        joint_command_data_.position.push_back(&(joint_it->second->command.position));
        joint_command_data_.velocity.push_back(&(joint_it->second->command.velocity));
        joint_command_data_.effort.push_back(&(joint_it->second->command.effort));
    }

    for (int i = 0; i < actuators.size(); ++i)
    {
        auto actuator = actuator_interface.getActuator(actuators[i]);

        actuator_state_data_.position.push_back(&(actuator->state.position));
        actuator_state_data_.velocity.push_back(&(actuator->state.velocity));
        actuator_state_data_.effort.push_back(&(actuator->state.effort));

        actuator_command_data_.position.push_back(&(actuator->command.position));
        actuator_command_data_.velocity.push_back(&(actuator->command.velocity));
        actuator_command_data_.effort.push_back(&(actuator->command.effort));
    }

    actuator_to_joint_ = std::unique_ptr<ActuatorToJointStateHandle>(
        new ActuatorToJointStateHandle(transmission_name, transmission_type.get(),
                                       actuator_state_data_, joint_state_data_));
    
    joint_to_actuator_ = std::unique_ptr<JointToActuatorStateHandle>(
        new JointToActuatorStateHandle(transmission_name, transmission_type.get(),
                                       actuator_command_data_, joint_command_data_));

    ActuatorToJointStateInterface &actuator_joint_interface = accessor.getActuatorToJointStateInterface();
    actuator_joint_interface.registerHandle(*actuator_to_joint_);

    JointToActuatorStateInterface &joint_actuator_interface = accessor.getJointToActuatorStateInterface();
    joint_actuator_interface.registerHandle(*joint_to_actuator_);
}