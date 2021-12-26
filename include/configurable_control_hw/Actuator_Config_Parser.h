#ifndef __ACTUATOR_CONFIG_PARSER__
#define __ACTUATOR_CONFIG_PARSER__

#include "Actuator_Controller.h"
#include <xmlrpcpp/XmlRpcValue.h>

// common base class for all actuator and contoller config parsers
class Actuator_Config_Parser
{
public:
    virtual void init(bool& stop_flag)
    {
        stop_flag_ptr_ = &stop_flag;
    }

    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Actuator_Controller_Map map) = 0;
protected:
    bool* stop_flag_ptr_;
};
#endif