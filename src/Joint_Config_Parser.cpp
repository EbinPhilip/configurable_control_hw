#include "Joint_Config_Parser.h"

#include <string>
#include <memory>

#include "rotational_units/Rotational_Units.h"

using namespace XmlRpc;

void Joint_Config_Parser::parseConfig(XmlRpc::XmlRpcValue& config, Joint_Map map)
{
    if (config.getType() != XmlRpcValue::Type::TypeStruct)
    {
        throw std::runtime_error("joint config parsing failed!");
    }

    if (!config.hasMember("joints"))
    {
        throw std::runtime_error("joint config not found!");
    }
    XmlRpcValue& joint_config = config["joints"];
    if (joint_config.getType() != XmlRpcValue::Type::TypeStruct)
    {
        throw std::runtime_error("joint config parsing failed!");
    }

    for(auto it = joint_config.begin(); it != joint_config.end(); ++it)
    {
        std::string name = it->first;
        std::string type = it->second["type"];
        Joint_Properties_Ptr joint_properties;

        if (type == "PositionControlledJoint")
        {
            if (it->second.hasMember("default_rpm"))
            {
                RUnits::RPM rpm = static_cast<double>(it->second["default_rpm"]);
                joint_properties = std::make_shared<Position_Joint_Properties>(name, rpm);
                
            }
            else
            {
                joint_properties = std::make_shared<Position_Joint_Properties>(name);
            }

            joint_properties->setDefaultVelocity();
        }
        else if (type == "PosVelControlledJoint")
        {
            joint_properties = std::make_shared<PosVel_Joint_Properties>(name);
        }
        else
        {
            throw std::runtime_error("unknown joint type: "+type);
        }

        map->insert(std::make_pair(name, joint_properties));
    }
}