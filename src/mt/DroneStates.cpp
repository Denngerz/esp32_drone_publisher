#include "../../include/mt/DroneStates.hpp"
#include <cmath>

namespace
{
    float sign(float v) { return v >= 0.0f ? 1.0f : -1.0f; }
}

// Stopped: turn toward the goal if mis-aligned, otherwise start accelerating.
std::unique_ptr<IMtState> MtStopped::execute(MtContext& ctx)
{
    float desired = angleOf(ctx.goal - ctx.position);
    float delta   = normalizeAngle(desired - ctx.direction);

    if (std::fabs(delta) > ctx.cfg.turnThreshold)
    {
        ctx.targetDir = desired;
        ctx.command   = { DroneState::TURNING, sign(delta) * ctx.cfg.angularSpeed };
        return std::make_unique<MtTurning>();
    }

    ctx.command = { DroneState::ACCELERATING, 0.0f };
    return std::make_unique<MtAccelerating>();
}

// Turning: keep rotating until the heading lines up with the goal.
std::unique_ptr<IMtState> MtTurning::execute(MtContext& ctx)
{
    float desired = angleOf(ctx.goal - ctx.position);
    float delta   = normalizeAngle(desired - ctx.direction);

    if (std::fabs(delta) <= ctx.cfg.turnThreshold)
    {
        ctx.command = { DroneState::ACCELERATING, 0.0f };
        return std::make_unique<MtAccelerating>();
    }

    ctx.command = { DroneState::TURNING, sign(delta) * ctx.cfg.angularSpeed };
    return nullptr;
}

// Accelerating: spin up to attackSpeed, then cruise.
std::unique_ptr<IMtState> MtAccelerating::execute(MtContext& ctx)
{
    if (ctx.speed >= ctx.cfg.attackSpeed - 1e-3f)
    {
        ctx.command = { DroneState::MOVING, 0.0f };
        return std::make_unique<MtMoving>();
    }

    ctx.command = { DroneState::ACCELERATING, 0.0f };
    return nullptr;
}

// Moving: cruise until close to the goal or a heading correction is needed.
std::unique_ptr<IMtState> MtMoving::execute(MtContext& ctx)
{
    float desired = angleOf(ctx.goal - ctx.position);
    float delta   = normalizeAngle(desired - ctx.direction);
    float dist    = length(ctx.goal - ctx.position);

    if (dist <= ctx.brakingDistance() + ctx.cfg.hitRadius ||
        std::fabs(delta) > ctx.cfg.turnThreshold)
    {
        ctx.command = { DroneState::DECELERATING, 0.0f };
        return std::make_unique<MtDecelerating>();
    }

    ctx.command = { DroneState::MOVING, 0.0f };
    return nullptr;
}

// Decelerating: brake to a stop.
std::unique_ptr<IMtState> MtDecelerating::execute(MtContext& ctx)
{
    if (ctx.speed <= 1e-3f)
    {
        ctx.command = { DroneState::STOPPED, 0.0f };
        return std::make_unique<MtStopped>();
    }

    ctx.command = { DroneState::DECELERATING, 0.0f };
    return nullptr;
}
