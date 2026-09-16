#pragma once
#include <memory>
#include "../dto/DroneContext.hpp"

// Base class for the drone state machine. Each former switch/case branch
// becomes a concrete state. execute() advances the drone one step and
// returns the next state, or nullptr to stay in the current one.
class IDroneState
{
public:
    virtual ~IDroneState() = default;

    virtual std::unique_ptr<IDroneState> execute(DroneContext& ctx) = 0;

    virtual const char* name() const = 0;
};
