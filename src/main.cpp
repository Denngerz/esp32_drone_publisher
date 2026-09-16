#include "../include/ComponentsFabric.hpp"
#include "../include/interfaces/IBallisticSolver.hpp"
#include "../include/interfaces/ITargetProvider.hpp"
#include "../include/interfaces/IConfigLoader.hpp"
#include "../include/MissionProcessor.hpp"
#include <iostream>
#include <memory>
#include <string>

int main(int argc, char** argv)
{
    ComponentsFabric fabric;

    // Optional first argument: "table" picks the table-based solver,
    // anything else (default) uses the analytical one.
    SolverType solverType = SolverType::ANALYTICAL;
    if (argc > 1 && std::string(argv[1]) == "table")
        solverType = SolverType::TABLE;

    // Components are owned through unique_ptr and handed to the processor
    // via std::move; nothing is deleted by hand.
    auto solver  = fabric.createSolver(solverType);
    auto targets = fabric.createProvider(ProviderType::JSON);
    auto loader  = fabric.createLoader(LoaderType::FILE);

    MissionProcessor mission(std::move(solver),
                             std::move(targets),
                             std::move(loader));

    if (!mission.init("data/config.json", "data/ammo.json", "data/targets.json"))
    {
        std::cerr << "Failed to initialize mission\n";
        return 1;
    }

    while (mission.hasNext())
        mission.step();

    if (!mission.writeLog("simulation.json"))
        std::cerr << "Failed to write simulation.json\n";

    std::cout << "Mission complete -> simulation.json\n";
    return 0;
}
