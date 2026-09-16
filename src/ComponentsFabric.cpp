#include "../include/ComponentsFabric.hpp"
#include "../include/AnalyticalSolver.hpp"
#include "../include/TableSolver.hpp"
#include "../include/JsonTargetProvider.hpp"
#include "../include/FileConfigLoader.hpp"

std::unique_ptr<IBallisticSolver> ComponentsFabric::createSolver(SolverType type)
{
    switch (type)
    {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalSolver>();
        case SolverType::TABLE:
            return std::make_unique<TableSolver>();
        default:
            return nullptr;
    }
}

std::unique_ptr<ITargetProvider> ComponentsFabric::createProvider(ProviderType type)
{
    switch (type)
    {
        case ProviderType::JSON:
            return std::make_unique<JsonTargetProvider>();
        default:
            return nullptr;
    }
}

std::unique_ptr<IConfigLoader> ComponentsFabric::createLoader(LoaderType type)
{
    switch (type)
    {
        case LoaderType::FILE:
            return std::make_unique<FileConfigLoader>();
        default:
            return nullptr;
    }
}
