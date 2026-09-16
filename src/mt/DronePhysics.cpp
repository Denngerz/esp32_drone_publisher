#include "../../include/mt/DronePhysics.hpp"
#include <chrono>

DronePhysics::DronePhysics(const DroneConfig& cfg, Coord startPos, float startDir)
    : cfg_(cfg), pos_(startPos), direction_(startDir)
{
    publish();
}

float DronePhysics::acceleration() const
{
    const float v = cfg_.attackSpeed;
    const float s = cfg_.accelPath > 0.0f ? cfg_.accelPath : 1.0f;
    return (v * v) / (2.0f * s);
}

void DronePhysics::sendCommand(const DroneCommand& cmd)
{
    commands_.push(cmd);
}

void DronePhysics::applyLatestCommand()
{
    if (auto cmd = commands_.drainLatest())
    {
        mode_       = cmd->state;
        angleSpeed_ = cmd->angleSpeed;
    }
}

void DronePhysics::step(float dt)
{
    applyLatestCommand();

    const float a = acceleration();

    switch (mode_)
    {
        case DroneState::TURNING:
            direction_ = normalizeAngle(direction_ + angleSpeed_ * dt);
            break;

        case DroneState::ACCELERATING:
            speed_ += a * dt;
            if (speed_ > cfg_.attackSpeed) speed_ = cfg_.attackSpeed;
            pos_ = pos_ + dirVec(direction_) * (speed_ * dt);
            break;

        case DroneState::MOVING:
            pos_ = pos_ + dirVec(direction_) * (speed_ * dt);
            break;

        case DroneState::DECELERATING:
            speed_ -= a * dt;
            if (speed_ < 0.0f) speed_ = 0.0f;
            pos_ = pos_ + dirVec(direction_) * (speed_ * dt);
            break;

        case DroneState::STOPPED:
            speed_ = 0.0f;
            break;
    }

    timeSinceStart_ += dt;
    publish();
}

void DronePhysics::publish()
{
    DroneTelemetry t;
    t.pos               = pos_;
    t.speed             = dirVec(direction_) * speed_;
    t.direction         = direction_;
    t.state             = mode_;
    t.timeSecSinceStart = timeSinceStart_;

    std::lock_guard<std::mutex> lock(teleMutex_);
    telemetry_ = t;
}

DroneTelemetry DronePhysics::getTelemetry() const
{
    std::lock_guard<std::mutex> lock(teleMutex_);
    return telemetry_;
}

void DronePhysics::start()
{
    started_.store(true);
}

void DronePhysics::stop()
{
    running_.store(false);
}

void DronePhysics::run()
{
    ready_.store(true);

    const float dt    = cfg_.physicsTimeStep;
    const float scale = cfg_.timeScale > 0.0f ? cfg_.timeScale : 1.0f;

    while (running_.load())
    {
        if (started_.load())
            step(dt);

        std::this_thread::sleep_for(std::chrono::duration<float>(dt / scale));
    }
}
