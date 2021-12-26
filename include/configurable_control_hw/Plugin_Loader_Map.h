#ifndef __PLUGIN_LOADER_MAP_H__
#define __PLUGIN_LOADER_MAP_H__

#include <map>
#include <memory>

#include <pluginlib/class_loader.h>

template <typename BaseClass>
class Plugin_Loader_Map
{
protected:
    std::map<std::pair<std::string, std::string>,
             std::shared_ptr<pluginlib::ClassLoader<BaseClass>>>
        plugin_map_;

public:
    std::shared_ptr<pluginlib::ClassLoader<BaseClass>>
    fetchPluginLoader(std::string package,
                      std::string base_class)
    {
        auto key_pair = std::make_pair(package, base_class);
        if (plugin_map_.find(key_pair) != plugin_map_.end())
        {
            return plugin_map_[key_pair];
        }
        else
        {
            auto loader_ptr = std::make_shared<pluginlib::ClassLoader<BaseClass>>(package, base_class);
            plugin_map_.insert(std::make_pair(key_pair, loader_ptr));
            return loader_ptr;
        }
    }
};

#endif