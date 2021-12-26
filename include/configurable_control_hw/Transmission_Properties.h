#ifndef __TRANSMISSION_PROPERTIES_H__
#define __TRANSMISSION_PROPERTIES_H__

#include "Joint_Properties.h"

#include <transmission_interface/transmission_interface.h>

#include <memory>
#include <string>

struct Transmission_Interfaces_Accessor;
class Actuator_Control_Interface;

// keeps track of tranmission-joint-actuator relationships, transmission interfaces and interface registration
struct Transmission_Properties
{
protected:
    typedef std::shared_ptr<transmission_interface::Transmission> Transmission_Ptr;
    typedef std::unique_ptr<transmission_interface::ActuatorToJointStateHandle> ActuatorToJointState_Ptr;
    typedef std::unique_ptr<transmission_interface::JointToActuatorStateHandle> JointToActuatorState_Ptr;

    ActuatorToJointState_Ptr actuator_to_joint_;
    JointToActuatorState_Ptr joint_to_actuator_;

    transmission_interface::JointData joint_state_data_;
    transmission_interface::ActuatorData actuator_state_data_;

    transmission_interface::JointData joint_command_data_;
    transmission_interface::ActuatorData actuator_command_data_;

public:
    Transmission_Properties(const std::string &, Transmission_Ptr);
    void registerTransmission(Transmission_Interfaces_Accessor &,
                              Actuator_Control_Interface &,
                              Joint_Map);

    std::string transmission_name;
    Transmission_Ptr transmission_type;

    std::vector<std::string> joints;
    std::vector<std::string> actuators;
};

typedef std::shared_ptr<Transmission_Properties> Transmission_Properties_Ptr;
typedef std::shared_ptr<std::map<std::string,Transmission_Properties_Ptr>> Transmissions_Map;

#endif