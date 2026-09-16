#pragma once
#include "../dto/AmmoParams.hpp"
 
struct Ballistics
{
    double hLength;
    double flightTime;
};
 
class IBallisticSolver
{
public:
    virtual Ballistics solve(float altitude, float speed, const AmmoParams& ammo) = 0;
    virtual ~IBallisticSolver() = default;
};