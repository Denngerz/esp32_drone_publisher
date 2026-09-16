#pragma once
#include "interfaces/IConfigLoader.hpp"
#include <string>
#include <unordered_map>
 
class FileConfigLoader : public IConfigLoader
{
public:
    bool        load(const char* configPath, const char* ammoPath) override;
    DroneConfig getConfig() override;
    AmmoParams  getAmmoParams(const char* name) override;
 
private:
    DroneConfig                              config_;
    std::unordered_map<std::string, AmmoParams> ammoMap_;  // was: AmmoParams* ammo_ + int ammoCount_
};