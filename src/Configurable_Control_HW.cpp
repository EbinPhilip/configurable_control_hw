#include "Configurable_Control_HW.h"

#include <xmlrpcpp/XmlRpcValue.h>
#include <ros/ros.h>
#include <ros/console.h>

#include <memory>
#include <stdexcept>
#include <string>

using namespace XmlRpc;

Configurable_Control_HW::Configurable_Control_HW()
    : write_enabled_(false),
      read_count_(0),
      stop_flag_(false),
      joint_map_(std::make_shared<std::map<std::string, Joint_Properties_Ptr>>()),
      transmissions_map_(std::make_shared<std::map<std::string, Transmission_Properties_Ptr>>()),
      controller_map_(std::make_shared<std::map<std::string, Actuator_Controller_Ptr>>()),
      actuator_interface_(nullptr)
{
    actuator_controller_creator_.init(stop_flag_);
}

bool Configurable_Control_HW::init(ros::NodeHandle &root_nh, ros::NodeHandle &nh)
{
    XmlRpcValue value;
    nh.getParam("", value);

    joint_parser_.parseConfig(value, joint_map_);

    transmission_parser_.parseConfig(value, transmissions_map_);

    Actuator_Controller_Map controller_map = std::make_shared<std::map<std::string, Actuator_Controller_Ptr>>();
    actuator_controller_creator_.parseConfig(value, controller_map);
    actuator_interface_ = std::make_shared<Actuator_Control_Interface>(controller_map, stop_flag_);

    for (auto &it : *transmissions_map_)
    {
        it.second->registerTransmission(*this, *actuator_interface_, joint_map_);
    }

    for (auto &it : *joint_map_)
    {
        it.second->registerJointInterfaces(*this);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
    registerInterface(&pos_vel_joint_interface_);

    enable_actuators_service_ = nh.advertiseService("enableActuators", &Configurable_Control_HW::_enableActuatorsServiceCallback, this);

    return true;
}

void Configurable_Control_HW::read()
{
    _handleEmergencyStop();
    actuator_interface_->readState();
    actuator_to_joint_interface_.propagate();
    read_count_++;
}

void Configurable_Control_HW::write()
{
    _handleEmergencyStop();
    if (!write_enabled_)
    {
        return;
    }
    for (auto &it : *joint_map_)
    {
        it.second->setDefaultVelocity();
    }
    joint_to_actuator_interface_.propagate();
    actuator_interface_->writeCommand();
}

hardware_interface::JointStateInterface &Configurable_Control_HW::getJointStateInterface()
{
    return joint_state_interface_;
}

hardware_interface::PositionJointInterface &Configurable_Control_HW::getPositionJointInterface()
{
    return position_joint_interface_;
}

hardware_interface::PosVelJointInterface &Configurable_Control_HW::getPosVelJointInterface()
{
    return pos_vel_joint_interface_;
}

transmission_interface::ActuatorToJointStateInterface &Configurable_Control_HW::getActuatorToJointStateInterface()
{
    return actuator_to_joint_interface_;
}

transmission_interface::JointToActuatorStateInterface &Configurable_Control_HW::getJointToActuatorStateInterface()
{
    return joint_to_actuator_interface_;
}

void Configurable_Control_HW::_handleEmergencyStop()
{
    if (stop_flag_)
    {
        bool error_status;
        std::string error_description;
        actuator_interface_->disableActuators();
        error_status = actuator_interface_->getErrorDetails(error_description);

        ROS_WARN("Emergency stop called!");
        if (error_status)
        {
            throw std::runtime_error(error_description);
        }
    }
}

bool Configurable_Control_HW::_enableActuatorsServiceCallback(
    configurable_control_hw::enableActuatorsRequest &req,
    configurable_control_hw::enableActuatorsResponse &res)
{
    int wait_count = 0;
    // wait for 5 reads to complete
    while(read_count_<5 && wait_count<5)
    {
        ros::Duration(1).sleep();
        wait_count++;
    }
    if(read_count_<5 && wait_count>=5)
    {
        res.result = false;
        return false;
    }
    long long read_count = read_count_;
    
    if (!write_enabled_)
    {
        for(auto& joint_it : *joint_map_)
        {
            joint_it.second->command.position = joint_it.second->state.position;
            joint_it.second->command.velocity = joint_it.second->state.velocity;
            joint_it.second->command.effort = joint_it.second->state.effort;
        }

        // wait for another 5 reads
        wait_count = 0;
        while(read_count_-read_count<5 && wait_count<5)
        {
            ros::Duration(1).sleep();
            wait_count++;
        }
        if(read_count_-read_count<5 && wait_count>=5)
        {
            res.result = false;
            return false;
        }
        
        if (!stop_flag_)
        {
            actuator_interface_->enableActuators();
            write_enabled_ = true;
        }
        else
        {
            _handleEmergencyStop();
        }
    }
    res.result = true;
    return true;
}