#pragma once
#include <memory>

class IBallisticSolver;
class IConfigLoader;

enum class SolverType { ANALYTICAL, TABLE };
enum class LoaderType { FILE };

class ComponentsFabric
{
public:
    std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
    std::unique_ptr<IConfigLoader>    createLoader(LoaderType type);
};
