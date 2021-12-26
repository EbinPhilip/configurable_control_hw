#include <pluginlib/class_loader.h>

#include "Actuator_Controllers_Creator.h"
// #include "G15_Actuator_Config_Parser.h"
// #include "G15_Controller_Config_Parser.h"

#include <memory>
#include <string>
#include <stdexcept>

using namespace XmlRpc;

void Actuator_Controllers_Creator::parseConfig(XmlRpc::XmlRpcValue &config, Actuator_Controller_Map controller_map)
{
    if (config.getType() != XmlRpcValue::Type::TypeStruct)
    {
        throw std::runtime_error("invalid actuator/controller config structure!");
    }

    _createActuatorControllers(config, controller_map);
    _addActuators(config, controller_map);
}

void Actuator_Controllers_Creator::_createActuatorControllers(XmlRpc::XmlRpcValue &config, Actuator_Controller_Map controller_map)
{
    auto controllers_list = _getEntityListFromConfig(config, "controllers");
    _createEntityFromConfig(controllers_list, controller_map);
}

void Actuator_Controllers_Creator::_addActuators(XmlRpc::XmlRpcValue &config, Actuator_Controller_Map controller_map)
{
    auto actuators_list = _getEntityListFromConfig(config, "actuators");
    _createEntityFromConfig(actuators_list, controller_map);
}

void Actuator_Controllers_Creator::_createEntityFromConfig(XmlRpc::XmlRpcValue &config, Actuator_Controller_Map controller_map)
{
    if (config.getType() != XmlRpcValue::Type::TypeArray)
    {
        throw std::runtime_error("invalid actuator/controller config structure!");
    }
    for (int i = 0; i < config.size(); ++i)
    {
        std::string package_name = config[i]["package"];
        std::string class_name = config[i]["class"];
        XmlRpc::XmlRpcValue &config_map = config[i]["config"];
        auto parser_loader = plugin_map_.fetchPluginLoader(package_name, "Actuator_Config_Parser");

        std::shared_ptr<Actuator_Config_Parser> actuator_creator(
            parser_loader->createUnmanagedInstance(class_name));
        actuator_creator->init(*stop_flag_ptr_);
        actuator_creator->parseConfig(config_map, controller_map);
    }
}

XmlRpcValue &Actuator_Controllers_Creator::_getEntityListFromConfig(XmlRpc::XmlRpcValue &config, const std::string &entity_name)
{
    if (!config.hasMember(entity_name))
    {
        throw std::runtime_error(entity_name + " not found!");
    }
    XmlRpcValue &controllers_list = config[entity_name];

    return controllers_list;
}
