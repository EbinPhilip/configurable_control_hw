#ifndef __JOINT_CONFIG_PARSER_H__
#define __JOINT_CONFIG_PARSER_H__

#include "Joint_Properties.h"
#include <xmlrpcpp/XmlRpcValue.h>

class Joint_Config_Parser
{
public:
    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Joint_Map map);
};

#endif