#pragma once
#include "interfaces/IBallisticSolver.hpp"
 
class AnalyticalSolver: public IBallisticSolver
{
public:
    Ballistics solve(float altitude, float speed, const AmmoParams& ammo) override;
};