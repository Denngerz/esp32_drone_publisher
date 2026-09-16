#pragma once
#include "Coord.hpp"
 
struct SimStep
{
    Coord pos;
    float direction;
    int   state;
    int   targetIdx;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};