#include "Configurable_Control_HW.h"

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
    // Setup
    ros::init(argc, argv, "configurable_control_hw_node");
    ros::NodeHandle nh(ros::this_node::getName());

    double loop_rate;
    if (!nh.getParam("loop_rate_hz", loop_rate))
    {
        throw new std::runtime_error("loop_rate_hz param not specified");
    }

    Configurable_Control_HW robot;
    robot.init(nh, nh);
    controller_manager::ControllerManager cm(&robot);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // Control loop
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(loop_rate); // Hz rate
    
    while (ros::ok())
    {
        const ros::Time     time   = ros::Time::now();
        const ros::Duration period = time - prev_time;
        
        robot.read();
        cm.update(time, period);
        robot.write();
        
        rate.sleep();
    }
    return 0;
}