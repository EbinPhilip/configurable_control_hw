#ifndef __TRANSMISSION_LOADER_PLUGIN_H__
#define __TRANSMISSION_LOADER_PLUGIN_H__

#include <memory>

#include <transmission_interface/transmission.h>
#include <xmlrpcpp/XmlRpcValue.h>

typedef std::shared_ptr<transmission_interface::Transmission> Transmission_Ptr;

class Transmission_Loader_Plugin
{
    public:
    virtual Transmission_Ptr loadTransmission(XmlRpc::XmlRpcValue& config) = 0;
};

#endif