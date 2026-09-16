#pragma once
#include "IDroneState.hpp"

// The five drone states. Each replaces one branch of the old switch/case.
//
//   Stopped       -> Turning or Accelerating (depending on heading error)
//   Turning       -> Accelerating (when aligned with the goal)
//   Accelerating  -> Moving (when attackSpeed is reached)
//   Moving        -> Decelerating (when approaching the goal / a turn)
//   Decelerating  -> Stopped (when speed reaches zero)

class StateStopped : public IDroneState
{
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override { return "Stopped"; }
};

class StateTurning : public IDroneState
{
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override { return "Turning"; }
};

class StateAccelerating : public IDroneState
{
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override { return "Accelerating"; }
};

class StateMoving : public IDroneState
{
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override { return "Moving"; }
};

class StateDecelerating : public IDroneState
{
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override { return "Decelerating"; }
};
