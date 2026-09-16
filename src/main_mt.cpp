#include "../include/ComponentsFabric.hpp"
#include "../include/interfaces/IBallisticSolver.hpp"
#include "../include/interfaces/IConfigLoader.hpp"
#include "../include/mt/DronePhysics.hpp"
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
    auto provider = std::make_unique<ThreadSafeTargetProvider>(
        cfg.arrayTimeStep, cfg.targetTimeStep, cfg.timeScale);
    if (!provider->loadFromFile("data/targets.json"))
    {
        std::cerr << "Failed to load targets\n";
        return 1;
    }

    auto physics = std::make_unique<DronePhysics>(cfg, cfg.startPos, cfg.initialDir);

    MissionRunner mission(cfg, ammo, std::move(solver),
                          physics.get(), provider.get());

    std::thread providerThread(&ThreadSafeTargetProvider::run, provider.get());
    std::thread physicsThread(&DronePhysics::run, physics.get());
    std::thread missionThread(&MissionRunner::run, &mission);

    // Wait until every thread is up before releasing them, so the simulation
    // starts synchronised.
    while (!provider->isThreadReady() ||
           !physics->isThreadReady()  ||
           !mission.isThreadReady())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    provider->start();
    physics->start();
    mission.start();

    missionThread.join();   // the mission is the only thread main waits on

    physics->stop();        // flag + join for the worker threads
    provider->stop();
    providerThread.join();
    physicsThread.join();

    if (!mission.writeLog("simulation.json"))
        std::cerr << "Failed to write simulation.json\n";

    std::cout << "Mission complete -> simulation.json\n";
    return 0;
}
