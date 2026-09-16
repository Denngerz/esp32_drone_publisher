#pragma once
#include "interfaces/IBallisticSolver.hpp"
#include "BallisticTable.hpp"
#include <string>

// IBallisticSolver backed by a pre-computed table instead of solving the
// equations of motion. Results between nodes use 5-D linear interpolation.
class TableSolver : public IBallisticSolver
{
public:
    explicit TableSolver(const char* tablePath = "data/ballistic_table.txt");

    Ballistics solve(float altitude, float speed, const AmmoParams& ammo) override;

private:
    BallisticTable table_;
    bool           loaded_ = false;
};
