#pragma once
// cadence.hpp — "is this due yet?", shared by both modules.
//
// Written so a 32-bit millisecond counter wrapping after about 49 days cannot
// leave a deadline permanently in the future: unsigned subtraction wraps with
// it, so the difference stays correct across the boundary.

#include <cstdint>

namespace cadence
{

inline bool due(uint32_t now, uint32_t last, uint32_t period)
{
    return static_cast<uint32_t>(now - last) >= period;
}

} // namespace cadence
