#pragma once
#include <memory>
#include "../dto/Coord.hpp"
#include "../dto/DroneConfig.hpp"
#include "../dto/DroneCommand.hpp"

// Shared data for the HW10 (command-based) state machine. Unlike the HW9
// version, these states do NOT integrate motion: they read a telemetry
// snapshot and emit a DroneCommand for the physics thread.
struct MtContext
{
    DroneConfig cfg;

    // Telemetry snapshot (input from physics).
    Coord position{ 0.0f, 0.0f };
    float direction = 0.0f;
    float speed     = 0.0f;

    Coord goal{ 0.0f, 0.0f };   // current drop point to fly to

    DroneCommand command;       // output to the physics thread
    float        targetDir = 0.0f;

    float acceleration() const
    {
        const float v = cfg.attackSpeed;
        const float s = cfg.accelPath > 0.0f ? cfg.accelPath : 1.0f;
        return (v * v) / (2.0f * s);
    }

    float brakingDistance() const
    {
        const float a = acceleration();
        return a > 0.0f ? (speed * speed) / (2.0f * a) : 0.0f;
    }
};

// Base class: decide the next state and set ctx.command. Return nullptr to
// stay in the current state.
class IMtState
{
public:
    virtual ~IMtState() = default;
    virtual std::unique_ptr<IMtState> execute(MtContext& ctx) = 0;
    virtual const char* name() const = 0;
};

class MtStopped : public IMtState
{
public:
    std::unique_ptr<IMtState> execute(MtContext& ctx) override;
    const char* name() const override { return "Stopped"; }
};

class MtTurning : public IMtState
{
public:
    std::unique_ptr<IMtState> execute(MtContext& ctx) override;
    const char* name() const override { return "Turning"; }
};

class MtAccelerating : public IMtState
{
public:
    std::unique_ptr<IMtState> execute(MtContext& ctx) override;
    const char* name() const override { return "Accelerating"; }
};

class MtMoving : public IMtState
{
public:
    std::unique_ptr<IMtState> execute(MtContext& ctx) override;
    const char* name() const override { return "Moving"; }
};

class MtDecelerating : public IMtState
{
public:
    std::unique_ptr<IMtState> execute(MtContext& ctx) override;
    const char* name() const override { return "Decelerating"; }
};
