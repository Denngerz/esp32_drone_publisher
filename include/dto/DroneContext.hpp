#pragma once
#include "Coord.hpp"
#include "DroneConfig.hpp"

// Shared data passed by reference between the drone state classes.
// Holds everything the state machine needs to advance the drone one
// simulation step toward the current goal (the drop point of the target
// the mission is currently servicing).
struct DroneContext
{
    DroneConfig cfg;             // simulation / drone configuration

    Coord  position{ 0.0f, 0.0f };
    float  direction   = 0.0f;   // current heading, radians
    float  desiredDir  = 0.0f;   // heading toward the current goal
    float  targetDir   = 0.0f;   // heading we are turning toward
    float  speed       = 0.0f;   // current speed
    float  turnRemaining = 0.0f; // seconds left to finish the in-place turn

    Coord  goal{ 0.0f, 0.0f };   // point the drone is flying to (drop point)
    bool   hasGoal     = false;  // false once all targets are serviced

    float  dt          = 0.1f;   // integration step for this tick (seconds)

    // Acceleration derived so the drone reaches attackSpeed over accelPath
    // metres: v^2 = 2*a*s  =>  a = v^2 / (2*s).
    float acceleration() const
    {
        const float v = cfg.attackSpeed;
        const float s = cfg.accelPath > 0.0f ? cfg.accelPath : 1.0f;
        return (v * v) / (2.0f * s);
    }

    // Distance needed to brake from the current speed to a stop.
    float brakingDistance() const
    {
        const float a = acceleration();
        return a > 0.0f ? (speed * speed) / (2.0f * a) : 0.0f;
    }
};
