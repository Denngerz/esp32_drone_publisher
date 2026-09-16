#include "../include/ComponentsFabric.hpp"
#include "../include/interfaces/IBallisticSolver.hpp"
#include "../include/interfaces/IConfigLoader.hpp"
#include "../include/mt/DronePhysics.hpp"
#include "../include/interfaces/ITargetSource.hpp"
#include "../include/mt/ThreadSafeTargetProvider.hpp"
#include "../include/mt/MissionRunner.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

int main(int argc, char** argv)
{
    ComponentsFabric fabric;

    // Load configuration and ammo through the existing loader.
    auto loader = fabric.createLoader(LoaderType::FILE);
    if (!loader->load("data/config.json", "data/ammo.json"))
    {
        std::cerr << "Failed to load configuration\n";
        return 1;
    }
    DroneConfig cfg  = loader->getConfig();
    AmmoParams  ammo = loader->getAmmoParams(cfg.ammoName.c_str());

    SolverType solverType = SolverType::ANALYTICAL;
    if (argc > 1 && std::string(argv[1]) == "table")
        solverType = SolverType::TABLE;
    auto solver = fabric.createSolver(solverType);

    // Three independently owned components, each with its own thread.
    //
    // Targets are reached through ITargetSource from here on. The concrete
    // type is chosen once, in this block, and nothing downstream knows which
    // one it got — that is what lets a seeker on the serial link stand in for
    // the local trajectory file.
    auto provider = std::make_unique<ThreadSafeTargetProvider>(
        cfg.arrayTimeStep, cfg.targetTimeStep, cfg.timeScale);
    if (!provider->loadFromFile("data/targets.json"))
    {
        std::cerr << "Failed to load targets\n";
        return 1;
    }
    ITargetSource* source = provider.get();

    auto physics = std::make_unique<DronePhysics>(cfg, cfg.startPos, cfg.initialDir);

    MissionRunner mission(cfg, ammo, std::move(solver),
                          physics.get(), source);

    std::thread providerThread(&ITargetSource::run, source);
    std::thread physicsThread(&DronePhysics::run, physics.get());
    std::thread missionThread(&MissionRunner::run, &mission);

    // Wait until every thread is up before releasing them, so the simulation
    // starts synchronised.
    while (!source->isThreadReady() ||
           !physics->isThreadReady()  ||
           !mission.isThreadReady())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    source->start();
    physics->start();
    mission.start();

    missionThread.join();   // the mission is the only thread main waits on

    physics->stop();        // flag + join for the worker threads
    source->stop();
    providerThread.join();
    physicsThread.join();

    if (!mission.writeLog("simulation.json"))
        std::cerr << "Failed to write simulation.json\n";

    std::cout << "Mission complete -> simulation.json\n";
    return 0;
}
