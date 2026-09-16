#pragma once
#include "../dto/Coord.hpp"
 
class ITargetProvider
{
public:
    virtual int   getTargetCount() = 0;
    virtual Coord getTarget(int index) = 0;
    virtual Coord getTargetAt(int index, float time) = 0;
    virtual void  interpolateAll(float time) = 0;
    virtual ~ITargetProvider() = default;
};