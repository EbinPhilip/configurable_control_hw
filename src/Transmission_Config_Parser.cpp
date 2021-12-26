#include "Transmission_Config_Parser.h"
#include "transmission_interface/simple_transmission.h"

using namespace XmlRpc;
using namespace transmission_interface;

Transmission_Config_Parser::Transmission_Config_Parser()
    : transmission_loader_("transmission_interface", "transmission_interface::Transmission")
{
}

void Transmission_Config_Parser::parseConfig(XmlRpc::XmlRpcValue &config, Transmissions_Map transmissions_map)
{
    if (config.getType() != XmlRpcValue::Type::TypeStruct)
    {
        throw std::runtime_error("transmissions config parsing failed!");
    }

    if (!config.hasMember("transmissions"))
    {
        throw std::runtime_error("transmissions config not found!");
    }
    XmlRpcValue &transmissions_config = config["transmissions"];
    if (transmissions_config.getType() != XmlRpcValue::Type::TypeStruct)
    {
        throw std::runtime_error("transmissions config parsing failed!");
    }

    for (auto it = transmissions_config.begin(); it != transmissions_config.end(); ++it)
    {
        std::string name = it->first;
        std::string type = it->second["type"];

        // FIX ME
        std::shared_ptr<transmission_interface::Transmission> transmission = 
            std::make_shared<transmission_interface::SimpleTransmission>(1.0);

        Transmission_Properties_Ptr transmission_properties;
        transmission_properties = std::make_shared<Transmission_Properties>(name, transmission);

        XmlRpcValue &joints_list = it->second["joints"];
        for (int i = 0; i < joints_list.size(); ++i)
        {
            transmission_properties->joints.push_back(joints_list[i]);
        }

        XmlRpcValue &actuators_list = it->second["actuators"];
        for (int i = 0; i < actuators_list.size(); ++i)
        {
            transmission_properties->actuators.push_back(actuators_list[i]);
        }

        transmissions_map->insert(std::make_pair(name, transmission_properties));
    }
}