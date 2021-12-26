#ifndef __CONFIGURABLE_CONTROL_HW_H__
#define __CONFIGURABLE_CONTROL_HW_H__

#include "Actuator_Controllers_Creator.h"
#include "Actuator_Controller.h"
#include "Actuator_Control_Interface.h"
#include "Actuator_Properties.h"

#include "Joint_Config_Parser.h"
#include "Joint_Properties.h"
#include "Transmission_Config_Parser.h"
#include "Transmission_Properties.h"

#include "Hardware_Interface_Accessors.h"

#include "configurable_control_hw/enableActuators.h"

#include "ros/ros.h"

#include <hardware_interface/robot_hw.h>

class Configurable_Control_HW : public hardware_interface::RobotHW,
                                public Joint_Interfaces_Accessor,
                                public Transmission_Interfaces_Accessor

{
public:
    Configurable_Control_HW();

    virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &nh) override;

    virtual void read();
    virtual void write();

    virtual hardware_interface::JointStateInterface &getJointStateInterface() override;
    virtual hardware_interface::PositionJointInterface &getPositionJointInterface() override;
    virtual hardware_interface::PosVelJointInterface &getPosVelJointInterface() override;

    virtual transmission_interface::ActuatorToJointStateInterface &getActuatorToJointStateInterface() override;
    virtual transmission_interface::JointToActuatorStateInterface &getJointToActuatorStateInterface() override;

protected:
    virtual void _handleEmergencyStop();

    bool _enableActuatorsServiceCallback(configurable_control_hw::enableActuatorsRequest &req,
                                     configurable_control_hw::enableActuatorsResponse &res);
    ros::ServiceServer enable_actuators_service_;
    bool write_enabled_;
    unsigned long long read_count_;

    bool stop_flag_;

    Joint_Config_Parser joint_parser_;
    Joint_Map joint_map_;

    Transmission_Config_Parser transmission_parser_;
    Transmissions_Map transmissions_map_;
    Actuator_Controller_Map controller_map_;

    Actuator_Controllers_Creator actuator_controller_creator_;
    Actuator_Control_Interface_Ptr actuator_interface_;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::PosVelJointInterface pos_vel_joint_interface_;

    transmission_interface::ActuatorToJointStateInterface actuator_to_joint_interface_;
    transmission_interface::JointToActuatorStateInterface joint_to_actuator_interface_;
};

#endif