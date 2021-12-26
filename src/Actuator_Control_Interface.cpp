#include "Actuator_Control_Interface.h"
#include <ros/console.h>

Actuator_Control_Interface::Actuator_Control_Interface(Actuator_Controller_Map controller_map,
                                                       bool &stop_flag, const std::string &instance_name)
    : controller_map_(controller_map),
      stop_flag_(stop_flag),
      error_status_(false),
      instance_name_(instance_name)
{
    for (auto &controller : *controller_map_)
    {
        std::vector<std::string> actuator_names;
        controller.second->getActuatorNames(actuator_names);

        for (auto actuator : actuator_names)
        {
            actuator_name_controller_map_.insert(std::make_pair(actuator, controller.second));
        }
    }
}

Actuator_Control_Interface::~Actuator_Control_Interface()
{
    disableActuators();
}

void Actuator_Control_Interface::readState()
{
    if (stop_flag_)
    {
        return;
    }
    for (auto &it : *controller_map_)
    {
        it.second->readState();
    }
}

void Actuator_Control_Interface::writeCommand()
{
    if (stop_flag_)
    {
        return;
    }
    for (auto &it : *controller_map_)
    {
        it.second->writeCommand();
    }
}

void Actuator_Control_Interface::enableActuators()
{
    if (stop_flag_)
    {
        return;
    }
    for (auto &it : *controller_map_)
    {
        it.second->enableActuators();
    }
}

void Actuator_Control_Interface::disableActuators()
{
    for (auto &it : *controller_map_)
    {
        it.second->disableActuators();
    }
    ROS_WARN("%s: actuators disabled", instance_name_.c_str());
}

bool Actuator_Control_Interface::getErrorDetails(std::string &error_msg)
{
    for (auto &it : *controller_map_)
    {
        error_status_ = error_status_ || it.second->getErrorDetails(error_msg);
    }
    return error_status_;
}

Actuator_Properties_Ptr Actuator_Control_Interface::getActuator(const std::string &name)
{
    auto it = actuator_name_controller_map_.find(name);
    if (it != actuator_name_controller_map_.end())
    {
        return it->second->getActuator(name);
    }
    else
    {
        return nullptr;
    }
}

void Actuator_Control_Interface::getActuatorNames(std::vector<std::string> &names)
{
    for (auto &it : actuator_name_controller_map_)
    {
        names.push_back(it.first);
    }
}