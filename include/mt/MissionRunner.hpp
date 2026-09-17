#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "../interfaces/IBallisticSolver.hpp"
#include "../dto/DroneConfig.hpp"
#include "../dto/AmmoParams.hpp"
#include "../dto/Coord.hpp"
#include "DronePhysics.hpp"
#include "../interfaces/ITargetSource.hpp"
#include "DroneStates.hpp"

// Runs the mission logic on its own thread: chooses the target, computes the
// ballistic drop point, drives the state machine, and commands the physics
// thread. It never stores or integrates the drone state itself — it only
// reads telemetry.
class MissionRunner
{
public:
    MissionRunner(const DroneConfig& cfg,
                  const AmmoParams& ammo,
                  std::unique_ptr<IBallisticSolver> solver,
                  DronePhysics* physics,
                  ITargetSource* targets);

    void run();              // thread body

    // True when run() stopped because the target source went silent rather
    // than because the mission finished. The caller decides what that means;
    // returning it keeps the exit status honest.
    bool aborted() const { return aborted_.load(); }
    bool isThreadReady() const { return ready_.load(); }
    void start();            // begin the mission
    void stop();             // external stop request

    bool writeLog(const char* path) const;

private:
    struct Step
    {
        Coord       position;
        float       direction;
        std::string state;
        int         targetIndex;
        Coord       dropPoint;
        Coord       aimPoint;
        Coord       predictedTarget;
        float       timeSecSinceStart;
    };

    void planAndCommand();   // one mission tick

    DroneConfig cfg_;
    AmmoParams  ammo_;
    std::unique_ptr<IBallisticSolver> solver_;
    DronePhysics*             physics_;
    ITargetSource*            targets_;

    std::unique_ptr<IMtState> state_;
    MtContext                 ctx_;
    int                       currentIdx_ = 0;

    std::vector<Step>  log_;
    mutable std::mutex logMutex_;

    std::atomic<bool> ready_{ false };
    std::atomic<bool> started_{ false };
    std::atomic<bool> running_{ true };
    std::atomic<bool> done_{ false };
    std::atomic<bool> aborted_{ false };

    static constexpr int MAX_STEPS = 200000;
};
