#ifndef __ACTUATOR_CONTROLLER_H__
#define __ACTUATOR_CONTROLLER_H__

#include "Actuator_Properties.h"

#include <string>
#include <vector>
#include <memory>
#include <map>

// base class for all actuator controllers
class Actuator_Controller
{
public:
    virtual void readState() = 0;
    virtual void writeCommand() = 0;

    virtual void enableActuators() = 0;
    virtual void disableActuators() = 0;

    virtual bool getErrorDetails(std::string&) = 0;

    virtual ::Actuator_Properties_Ptr getActuator(const std::string&) = 0;
    virtual void getActuatorNames(std::vector<std::string>&) = 0;
};

typedef std::shared_ptr<Actuator_Controller> Actuator_Controller_Ptr;
typedef std::shared_ptr< std::map<std::string, Actuator_Controller_Ptr> > Actuator_Controller_Map;

#endif