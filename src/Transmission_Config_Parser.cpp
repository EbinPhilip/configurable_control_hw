#include "Transmission_Config_Parser.h"
#include "transmission_interface/simple_transmission.h"

using namespace XmlRpc;
using namespace transmission_interface;

Transmission_Config_Parser::Transmission_Config_Parser()
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
        std::string class_name = it->second["type"];

        std::shared_ptr<transmission_interface::Transmission> transmission;

        if (class_name == "transmission_interface::SimpleTransmission")
        {
            transmission = std::make_shared<transmission_interface::SimpleTransmission>(1.0);
        }
        else
        {
            std::string package_name = it->second["package"];
            auto plugin_loader = plugin_map_.fetchPluginLoader(package_name, "Transmission_Loader_Plugin");
            std::shared_ptr<Transmission_Loader_Plugin> transmission_loader(
                    plugin_loader->createUnmanagedInstance(class_name));
            transmission = transmission_loader->loadTransmission(it->second);
        }

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