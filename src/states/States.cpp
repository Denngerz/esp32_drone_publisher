#include "../../include/states/States.hpp"
#include <cmath>

// ---------------------------------------------------------------------------
// Stopped: decide whether we first have to rotate toward the goal or can
// start accelerating straight away.
// ---------------------------------------------------------------------------
std::unique_ptr<IDroneState> StateStopped::execute(DroneContext& ctx)
{
    ctx.desiredDir = angleOf(ctx.goal - ctx.position);
    float delta = normalizeAngle(ctx.desiredDir - ctx.direction);

    if (std::fabs(delta) > ctx.cfg.turnThreshold)
    {
        ctx.turnRemaining = std::fabs(delta) / ctx.cfg.angularSpeed;
        ctx.targetDir = ctx.desiredDir;
        return std::make_unique<StateTurning>();
    }

    ctx.direction = ctx.desiredDir;
    return std::make_unique<StateAccelerating>();
}

// ---------------------------------------------------------------------------
// Turning: rotate in place toward targetDir at angularSpeed.
// ---------------------------------------------------------------------------
std::unique_ptr<IDroneState> StateTurning::execute(DroneContext& ctx)
{
    ctx.turnRemaining -= ctx.dt;

    float remaining = normalizeAngle(ctx.targetDir - ctx.direction);
    float maxStep   = ctx.cfg.angularSpeed * ctx.dt;

    if (std::fabs(remaining) <= maxStep || ctx.turnRemaining <= 0.0f)
    {
        ctx.direction = ctx.targetDir;
        return std::make_unique<StateAccelerating>();
    }

    ctx.direction = normalizeAngle(
        ctx.direction + (remaining > 0.0f ? maxStep : -maxStep));
    return nullptr;
}

// ---------------------------------------------------------------------------
// Accelerating: build up speed until attackSpeed, then cruise.
// ---------------------------------------------------------------------------
std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext& ctx)
{
    const float a = ctx.acceleration();
    bool reached = false;

    ctx.speed += a * ctx.dt;
    if (ctx.speed >= ctx.cfg.attackSpeed)
    {
        ctx.speed = ctx.cfg.attackSpeed;
        reached = true;
    }

    ctx.position = ctx.position + dirVec(ctx.direction) * (ctx.speed * ctx.dt);

    if (reached) return std::make_unique<StateMoving>();
    return nullptr;
}

// ---------------------------------------------------------------------------
// Moving: cruise at max speed; brake once close to the goal or a turn is
// needed.
// ---------------------------------------------------------------------------
std::unique_ptr<IDroneState> StateMoving::execute(DroneContext& ctx)
{
    ctx.position = ctx.position + dirVec(ctx.direction) * (ctx.speed * ctx.dt);

    ctx.desiredDir = angleOf(ctx.goal - ctx.position);
    float delta = normalizeAngle(ctx.desiredDir - ctx.direction);
    float dist  = length(ctx.goal - ctx.position);

    if (dist <= ctx.brakingDistance() ||
        std::fabs(delta) > ctx.cfg.turnThreshold)
    {
        return std::make_unique<StateDecelerating>();
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// Decelerating: brake down to a full stop.
// ---------------------------------------------------------------------------
std::unique_ptr<IDroneState> StateDecelerating::execute(DroneContext& ctx)
{
    const float a = ctx.acceleration();
    ctx.speed -= a * ctx.dt;

    if (ctx.speed <= 0.0f)
    {
        ctx.speed = 0.0f;
        return std::make_unique<StateStopped>();
    }

    ctx.position = ctx.position + dirVec(ctx.direction) * (ctx.speed * ctx.dt);
    return nullptr;
}
