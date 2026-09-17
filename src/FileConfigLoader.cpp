#include "../include/FileConfigLoader.hpp"
#include <fstream>
#include <iostream>
#include "nlohmann/json.hpp"
 
using json = nlohmann::json;
 
bool FileConfigLoader::load(const char* configPath, const char* ammoPath)
{
    // Load config
    std::ifstream cf(configPath);
    if (!cf.is_open())
    {
        std::cerr << "Cannot open " << configPath << '\n';
        return false;
    }
 
    json j;
    cf >> j;
 
    config_.startPos.x    = j["drone"]["position"]["x"];
    config_.startPos.y    = j["drone"]["position"]["y"];
    config_.altitude       = j["drone"]["altitude"];
    config_.initialDir     = j["drone"]["initialDirection"];
    config_.attackSpeed    = j["drone"]["attackSpeed"];
    config_.accelPath      = j["drone"]["accelerationPath"];
    config_.angularSpeed   = j["drone"]["angularSpeed"];
    config_.turnThreshold  = j["drone"]["turnThreshold"];
    config_.simTimeStep    = j["simulation"]["timeStep"];
    config_.hitRadius      = j["simulation"]["hitRadius"];
    config_.arrayTimeStep  = j["targetArrayTimeStep"];

    // HW10 parameters: optional, fall back to defaults when missing.
    config_.targetTimeStep  = j["simulation"].value("targetTimeStep", 0.05f);
    config_.physicsTimeStep = j["simulation"].value("physicsTimeStep", 0.01f);
    config_.timeScale       = j["simulation"].value("timeScale", 10.0f);
    config_.ammoName       = j["ammo"].get<std::string>();  // was: strncpy into char[32]
 
    // Load ammo
    std::ifstream af(ammoPath);
    if (!af.is_open())
    {
        std::cerr << "Cannot open " << ammoPath << '\n';
        return false;
    }
 
    json ja;
    af >> ja;
 
    ammoMap_.clear();
    for (const auto& entry : ja)
    {
        AmmoParams ap;
        ap.name = entry["name"].get<std::string>();
        ap.mass = entry["mass"];
        ap.drag = entry["drag"];
        ap.lift = entry["lift"];
        ammoMap_[ap.name] = ap;
    }
 
    return true;
}
 
DroneConfig FileConfigLoader::getConfig()
{
    return config_;
}
 
AmmoParams FileConfigLoader::getAmmoParams(const char* name)
{
    // was: for loop with strcmp over raw array
    auto it = ammoMap_.find(name);
    if (it == ammoMap_.end())
    {
        std::cerr << "Ammo not found: " << name << '\n';
        return {};
    }
    return it->second;
}