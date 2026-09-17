#pragma once
#include "../dto/DroneConfig.hpp"
#include "../dto/AmmoParams.hpp"
 
class IConfigLoader
{
public:
    virtual bool        load(const char* configPath, const char* ammoPath) = 0;
    virtual DroneConfig getConfig() = 0;
    virtual AmmoParams  getAmmoParams(const char* name) = 0;
    virtual ~IConfigLoader() = default;
};