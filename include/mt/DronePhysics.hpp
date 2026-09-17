#pragma once
#include <atomic>
#include <mutex>
#include <thread>
#include "../dto/Coord.hpp"
#include "../dto/DroneConfig.hpp"
#include "../dto/DroneCommand.hpp"
#include "../ThreadSafeQueue.hpp"

// Owns the drone's physical state (position, speed, heading) and integrates
// it on its own thread. The mission thread talks to it only through a command
// queue and immutable telemetry snapshots.
class DronePhysics
{
public:
    DronePhysics(const DroneConfig& cfg, Coord startPos, float startDir);

    // Queue a new command for the physics loop to pick up.
    void sendCommand(const DroneCommand& cmd);

    // Integrate one physics step of length dt (also callable synchronously,
    // without the thread, as recommended for incremental development).
    void step(float dt);

    // Immutable snapshot of the current drone state.
    DroneTelemetry getTelemetry() const;

    // Thread lifecycle.
    void run();              // thread body
    bool isThreadReady() const { return ready_.load(); }
    void start();            // begin integrating
    void stop();             // signal stop (the owner joins the thread)
    bool running() const { return running_.load(); }

private:
    void applyLatestCommand();

    DroneConfig cfg_;

    // Physical state (only touched by the physics thread / step()).
    Coord      pos_;
    float      direction_ = 0.0f;
    float      speed_     = 0.0f;
    DroneState mode_      = DroneState::STOPPED;
    float      angleSpeed_ = 0.0f;
    float      timeSinceStart_ = 0.0f;

    ThreadSafeQueue<DroneCommand> commands_;

    mutable std::mutex teleMutex_;
    DroneTelemetry     telemetry_;

    std::atomic<bool> ready_{ false };
    std::atomic<bool> started_{ false };
    std::atomic<bool> running_{ true };

    float acceleration() const;
    void  publish();
};
