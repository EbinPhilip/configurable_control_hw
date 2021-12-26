#ifndef __ACTUATOR_CONTROL_INTERFACE_H__
#define __ACTUATOR_CONTROL_INTERFACE_H__

#include "Actuator_Properties.h"
#include "Actuator_Controller.h"

#include <string>
#include <vector>
#include <memory>
#include <map>

// composes various actuator controllers and presents a common interface for all
class Actuator_Control_Interface : public Actuator_Controller
{
public:
    Actuator_Control_Interface(Actuator_Controller_Map controller_map, bool& stop_flag,
                                const std::string& instance_name = "control interface");
    ~Actuator_Control_Interface();

    virtual void readState() override;
    virtual void writeCommand() override;

    virtual void enableActuators() override;
    virtual void disableActuators() override;

    virtual bool getErrorDetails(std::string& error_msg) override;

    virtual Actuator_Properties_Ptr getActuator(const std::string&) override;
    virtual void getActuatorNames(std::vector<std::string>&) override;

protected:
    // map of controller names
    Actuator_Controller_Map controller_map_;
    std::string instance_name_;

    bool& stop_flag_;
    bool error_status_;

    // map of actuator name to controller name
    std::map<std::string, Actuator_Controller_Ptr> actuator_name_controller_map_;
};

typedef std::shared_ptr<Actuator_Control_Interface> Actuator_Control_Interface_Ptr;

#endif