#include "../include/TableSolver.hpp"
#include <iostream>

TableSolver::TableSolver(const char* tablePath)
{
    loaded_ = table_.load(tablePath);
    if (!loaded_)
        std::cerr << "TableSolver: failed to load " << tablePath << '\n';
}

Ballistics TableSolver::solve(float altitude, float speed, const AmmoParams& ammo)
{
    if (!loaded_)
        return { -1.0, -1.0 };

    BallisticTable::Result r =
        table_.lookup(altitude, speed, ammo.mass, ammo.drag, ammo.lift);

    return { (double)r.hDist, (double)r.t };
}
