#include "../include/ComponentsFabric.hpp"
#include "../include/interfaces/IBallisticSolver.hpp"
#include "../include/interfaces/IConfigLoader.hpp"
#include "../include/mt/DronePhysics.hpp"
#include "../include/interfaces/ITargetSource.hpp"
#include "../include/mt/SerialTargetSource.hpp"
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

    // Args: an optional "table" positional switches the solver, and
    // "--seeker <device>" takes targets and ammunition from the ESP32 over a
    // serial link instead of reading data/targets.json locally.
    SolverType  solverType = SolverType::ANALYTICAL;
    std::string seekerDevice;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "table")
            solverType = SolverType::TABLE;
        else if (arg == "--seeker" && i + 1 < argc)
            seekerDevice = argv[++i];
    }
    auto solver = fabric.createSolver(solverType);

    // Three independently owned components, each with its own thread.
    //
    // Targets are reached through ITargetSource from here on. The concrete
    // type is chosen once, in this block, and nothing downstream knows which
    // one it got — that is what lets a seeker on the serial link stand in for
    // the local trajectory file.
    std::unique_ptr<ThreadSafeTargetProvider> fileProvider;
    std::unique_ptr<SerialTargetSource>       seeker;
    ITargetSource*                            source = nullptr;

    if (seekerDevice.empty())
    {
        fileProvider = std::make_unique<ThreadSafeTargetProvider>(
            cfg.arrayTimeStep, cfg.targetTimeStep, cfg.timeScale);
        if (!fileProvider->loadFromFile("data/targets.json"))
        {
            std::cerr << "Failed to load targets\n";
            return 1;
        }
        source = fileProvider.get();
    }
    else
    {
        seeker = std::make_unique<SerialTargetSource>(seekerDevice);
        if (!seeker->openPort())
            return 1;
        source = seeker.get();
    }

    // The source's thread starts first and alone. With a seeker the mission's
    // own parameters arrive over the wire, so there is nothing to build a
    // MissionRunner out of until the bay has been heard from.
    std::thread providerThread(&ITargetSource::run, source);

    if (seeker)
    {
        if (!seeker->waitUntilReady(10000))
        {
            seeker->stop();
            providerThread.join();
            return 1;
        }

        // The store on the aircraft overrides whatever data/ammo.json and the
        // config's ammo name said: the bay reports what is physically there,
        // and its lethal radius is a property of the munition, not of the
        // simulation settings.
        ammo          = seeker->ammo();
        cfg.hitRadius = seeker->hitRadius();
    }

    auto physics = std::make_unique<DronePhysics>(cfg, cfg.startPos, cfg.initialDir);

    MissionRunner mission(cfg, ammo, std::move(solver),
                          physics.get(), source);

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
