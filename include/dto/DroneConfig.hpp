#pragma once
#include "Coord.hpp"
#include <string>
 
struct DroneConfig
{
    Coord       startPos;
    float       altitude;
    float       initialDir;
    float       attackSpeed;
    float       accelPath;
    std::string ammoName;
    float       arrayTimeStep;
    float       simTimeStep;
    float       hitRadius;
    float       angularSpeed;
    float       turnThreshold;

    // HW10 multithreading parameters (defaults applied if absent in config).
    float       targetTimeStep  = 0.05f;  // provider thread tick period
    float       physicsTimeStep = 0.01f;  // physics thread integration step
    float       timeScale       = 10.0f;  // real-time acceleration factor
};