#pragma once
#include "Coord.hpp"

// Drone movement mode. Reintroduced in HW10 purely as a label carried inside
// commands and telemetry between the mission and physics threads (the state
// *logic* still lives in the state classes, not in a switch/case).
enum class DroneState
{
    STOPPED,
    ACCELERATING,
    DECELERATING,
    TURNING,
    MOVING
};

inline const char* toString(DroneState s)
{
    switch (s)
    {
        case DroneState::STOPPED:      return "Stopped";
        case DroneState::ACCELERATING: return "Accelerating";
        case DroneState::DECELERATING: return "Decelerating";
        case DroneState::TURNING:      return "Turning";
        case DroneState::MOVING:       return "Moving";
    }
    return "Unknown";
}

// Command the mission thread sends to the physics thread.
struct DroneCommand
{
    DroneState state      = DroneState::STOPPED;
    float      angleSpeed = 0.0f;  // signed angular velocity while turning
};

// Snapshot the physics thread publishes for the mission thread.
struct DroneTelemetry
{
    Coord      pos{ 0.0f, 0.0f };
    Coord      speed{ 0.0f, 0.0f };   // velocity vector
    float      direction        = 0.0f;
    DroneState state            = DroneState::STOPPED;
    float      timeSecSinceStart = 0.0f;  // time of this physics update
};
