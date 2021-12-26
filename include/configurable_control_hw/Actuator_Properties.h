#ifndef __ACTUATOR_PROPERTIES_H__
#define __ACTUATOR_PROPERTIES_H__

#include "Control_Properties.h"

#include <transmission_interface/transmission.h>

#include <string>
#include <memory>

struct Actuator_Properties : public Controllable_Entity
{
    std::string actuator_name;
    std::string actuator_type;
};

typedef std::shared_ptr<Actuator_Properties> Actuator_Properties_Ptr;

#endif