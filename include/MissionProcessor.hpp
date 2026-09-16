#pragma once
#include <memory>
#include <string>
#include <vector>
#include "interfaces/IBallisticSolver.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/IConfigLoader.hpp"
#include "states/IDroneState.hpp"
#include "dto/DroneConfig.hpp"
#include "dto/AmmoParams.hpp"
#include "dto/DroneContext.hpp"
#include "dto/Coord.hpp"

struct DropResult
{
    Coord dropPoint;
    Coord targetPos;
    double hLength;
    double flightTime;
};

// One logged simulation step (serialised to simulation.json).
struct LoggedStep
{
    Coord       position;
    float       direction;
    std::string state;
    int         targetIndex;
    Coord       dropPoint;
    Coord       aimPoint;
    Coord       predictedTarget;
};

class MissionProcessor
{
public:
    MissionProcessor(std::unique_ptr<IBallisticSolver> solver,
                     std::unique_ptr<ITargetProvider>  targets,
                     std::unique_ptr<IConfigLoader>     loader);

    bool init(const char* configPath, const char* ammoPath, const char* targetsPath);
    bool hasNext();
    DropResult step();
    void reset();

    // Hand the processor a different solver (non-owning use also possible).
    void changeSolver(std::unique_ptr<IBallisticSolver> solver);

    // Write the accumulated steps to a simulation.json file.
    bool writeLog(const char* path) const;

private:
    std::unique_ptr<IBallisticSolver> solver_;
    std::unique_ptr<ITargetProvider>  targets_;
    std::unique_ptr<IConfigLoader>    loader_;

    DroneConfig config_;
    AmmoParams  ammo_;
    int         currentIdx_ = 0;

    std::unique_ptr<IDroneState> state_;
    DroneContext                 ctx_;
    float                        currentTime_ = 0.0f;
    int                          stepCount_   = 0;

    std::vector<LoggedStep>      log_;

    static constexpr int MAX_STEPS = 20000;

    // Recompute the intercept / drop point for the current target.
    DropResult planCurrentTarget();
};
