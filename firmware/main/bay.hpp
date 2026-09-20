#pragma once
// bay.hpp — the payload bay module.
//
// It reports what is physically loaded and the ballistic properties the
// flight computer needs to solve with. That is the whole of its job: a bay
// knows what is in the tube, and nothing about where anything is.
//
// It repeats itself rather than announcing once, so a flight computer that
// starts late, or misses a frame, still learns the store. Unlike the seeker's
// stream, the repetition carries no new information — it is there purely so a
// late listener catches up.
//
// This is a separate module from the seeker, on its own UART, with its own
// task. The two share nothing, which is the point: on an airframe they are
// separate boxes and they fail separately.
//
// Timing is passed in rather than read from a clock inside, which keeps the
// whole class testable on a host and free of FreeRTOS.

#include "mission_data.generated.hpp"

#include <cstdint>

namespace sensor_uart { class Uart; }

namespace bay
{

class Bay
{
public:
    Bay(sensor_uart::Uart& uart, const mission_data::Store& loaded)
        : uart_(uart), loaded_(loaded) {}

    // Call often; it decides internally what is due. nowMs is wall-clock
    // milliseconds since boot.
    void tick(uint32_t nowMs);

private:
    void publishAmmo();

    sensor_uart::Uart&         uart_;
    const mission_data::Store& loaded_;

    uint32_t lastAmmoMs_ = 0;
    bool     started_    = false;
};

} // namespace bay
