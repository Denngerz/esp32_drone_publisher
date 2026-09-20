#include "../include/ComponentsFabric.hpp"
#include "../include/interfaces/IBallisticSolver.hpp"
#include "../include/interfaces/IConfigLoader.hpp"
#include "../include/mt/DronePhysics.hpp"
#include "../include/interfaces/ITargetSource.hpp"
#include "../include/mt/SerialAmmoSource.hpp"
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

    // Args: an optional "table" positional switches the solver, and the two
    // device flags say which sensor modules are wired up.
    //
    //   --seeker <dev>   targets from the seeker instead of data/targets.json
    //   --bay <dev>      the payload bay as a separate module on its own link
    //
    // With --seeker alone one module publishes everything on one wire. Adding
    // --bay splits the two roles across two links, which is what they are on
    // an airframe: separate boxes that fail independently.
    SolverType  solverType = SolverType::ANALYTICAL;
    std::string seekerDevice;
    std::string bayDevice;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "table")
            solverType = SolverType::TABLE;
        else if (arg == "--seeker" && i + 1 < argc)
            seekerDevice = argv[++i];
        else if (arg == "--bay" && i + 1 < argc)
            bayDevice = argv[++i];
    }

    if (!bayDevice.empty() && seekerDevice.empty())
    {
        // A bay on its own reports a store for a mission that has no targets
        // to fly at, so this is a wiring mistake rather than a mode.
        std::cerr << "--bay needs --seeker: the bay reports the store, not the targets\n";
        return 1;
    }

    auto solver = fabric.createSolver(solverType);

    // Three independently owned components, each with its own thread, plus
    // the bay's reader when it is a module of its own.
    //
    // Targets are reached through ITargetSource from here on. The concrete
    // type is chosen once, in this block, and nothing downstream knows which
    // one it got — that is what lets a seeker on the serial link stand in for
    // the local trajectory file.
    std::unique_ptr<ThreadSafeTargetProvider> fileProvider;
    std::unique_ptr<SerialTargetSource>       seeker;
    std::unique_ptr<SerialAmmoSource>         bay;
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
        // With a separate bay, the seeker link carries no store report and
        // must not wait for one.
        seeker = std::make_unique<SerialTargetSource>(seekerDevice, 115200,
                                                      bayDevice.empty());
        if (!seeker->openPort())
            return 1;
        source = seeker.get();

        if (!bayDevice.empty())
        {
            bay = std::make_unique<SerialAmmoSource>(bayDevice);
            if (!bay->openPort())
                return 1;
        }
    }

    // The sensor threads start first and alone. With a seeker the mission's
    // own parameters arrive over the wire, so there is nothing to build a
    // MissionRunner out of until the modules have been heard from.
    std::thread providerThread(&ITargetSource::run, source);
    std::thread bayThread;
    if (bay)
        bayThread = std::thread(&SerialAmmoSource::run, bay.get());

    auto shutdownSensors = [&]()
    {
        source->stop();
        providerThread.join();
        if (bay)
        {
            bay->stop();
            bayThread.join();
        }
    };

    if (seeker)
    {
        if (!seeker->waitUntilReady(10000))
        {
            shutdownSensors();
            return 1;
        }

        // The store on the aircraft overrides whatever data/ammo.json and the
        // config's ammo name said: the bay reports what is physically there,
        // and its lethal radius is a property of the munition, not of the
        // simulation settings. Which module said so does not change that.
        if (bay)
        {
            if (!bay->waitUntilReady(10000))
            {
                shutdownSensors();
                return 1;
            }
            ammo          = bay->ammo();
            cfg.hitRadius = bay->hitRadius();
        }
        else
        {
            ammo          = seeker->ammo();
            cfg.hitRadius = seeker->hitRadius();
        }
    }

    auto physics = std::make_unique<DronePhysics>(cfg, cfg.startPos, cfg.initialDir);

    MissionRunner mission(cfg, ammo, std::move(solver),
                          physics.get(), source);

    std::thread physicsThread(&DronePhysics::run, physics.get());
    std::thread missionThread(&MissionRunner::run, &mission);

    // Wait until every thread is up before releasing them, so the simulation
    // starts synchronised. The bay's reader is not among them: it has already
    // said everything the mission needs, and nothing waits on it again.
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
    shutdownSensors();
    physicsThread.join();

    if (!mission.writeLog("simulation.json"))
        std::cerr << "Failed to write simulation.json\n";

    // The log is written either way: a run that lost the link partway is
    // still worth inspecting. Only the exit status distinguishes them.
    if (mission.aborted())
    {
        std::cerr << "Mission aborted -> simulation.json holds the partial run\n";
        return 1;
    }

    std::cout << "Mission complete -> simulation.json\n";
    return 0;
}
