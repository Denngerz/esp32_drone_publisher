#include "bay.hpp"
#include "board_config.hpp"
#include "cadence.hpp"
#include "sensor_uart.hpp"

#include "link/SensorLink.hpp"

#include <cstring>

namespace bay
{

void Bay::publishAmmo()
{
    sensor_link::AmmoReport a{};

    // The catalogue name may be shorter or longer than the wire field. Copy
    // what fits and leave the rest zeroed; the field is documented as not
    // necessarily NUL-terminated, so a name of exactly 16 characters is fine
    // and must not be truncated to make room for a terminator.
    const size_t nameLen = std::strlen(loaded_.name);
    std::memcpy(a.name, loaded_.name,
                nameLen < sizeof a.name ? nameLen : sizeof a.name);

    a.mass      = loaded_.mass;
    a.drag      = loaded_.drag;
    a.lift      = loaded_.lift;
    a.hitRadius = board::kStoreHitRadiusM;

    uart_.send(sensor_link::PKT_AMMO, &a, sizeof a);
}

void Bay::tick(uint32_t nowMs)
{
    // Report immediately on the first tick, then on the repeat cadence. The
    // flight computer blocks until it has heard this, so the first one should
    // not wait out a period.
    if (!started_)
    {
        started_ = true;
        publishAmmo();
        lastAmmoMs_ = nowMs;
        return;
    }

    if (cadence::due(nowMs, lastAmmoMs_, board::kAmmoRepeatMs))
    {
        lastAmmoMs_ = nowMs;
        publishAmmo();
    }
}

} // namespace bay
