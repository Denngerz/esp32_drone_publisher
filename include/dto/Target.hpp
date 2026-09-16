#pragma once
#include "Coord.hpp"

// HW10 target: only the current snapshot is exposed outside the provider;
// the full trajectory stays private to ThreadSafeTargetProvider.
struct Target
{
    Coord pos{ 0.0f, 0.0f };       // current position
    Coord velocity{ 0.0f, 0.0f };  // current velocity
};
