#ifndef __TRANSMISSION_CONFIG_PARSER_H__
#define __TRANSMISSION_CONFIG_PARSER_H__

#include "Transmission_Properties.h"
#include "Transmission_Loader_Plugin.h"
#include "Plugin_Loader_Map.h"

#include <pluginlib/class_loader.h>
#include <transmission_interface/transmission.h>
#include <xmlrpcpp/XmlRpcValue.h>

class Transmission_Config_Parser
{
public:
    Transmission_Config_Parser();
    virtual void parseConfig(XmlRpc::XmlRpcValue& config, Transmissions_Map map);
protected:
    Plugin_Loader_Map<Transmission_Loader_Plugin> plugin_map_;;
};

#endif