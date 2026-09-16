#include "publisher.hpp"
#include "board_config.hpp"
#include "sensor_uart.hpp"

#include "link/SensorLink.hpp"

#include <cmath>
#include <cstring>

namespace publisher
{

namespace
{
// Wall-clock milliseconds between two rounds of detections.
constexpr uint32_t kDetectionPeriodMs = 1000 / board::kDetectionHz;

// True when `now` has reached `deadline`, written so a 32-bit millisecond
// counter wrapping after about 49 days cannot leave a deadline permanently in
// the future.
bool due(uint32_t now, uint32_t last, uint32_t period)
{
    return static_cast<uint32_t>(now - last) >= period;
}
} // namespace

void Publisher::sampleTrack(int index, float missionTimeS,
                            float& x, float& y) const
{
    const auto& track = mission_data::kTracks[index];
    constexpr int n = mission_data::kNodeCount;

    const float nodes = missionTimeS / board::kNodeIntervalS;
    const float floorNodes = std::floor(nodes);

    int idx = static_cast<int>(floorNodes) % n;
    if (idx < 0) idx += n;
    const int next = (idx + 1) % n;

    const float frac = nodes - floorNodes;

    x = track[idx].x + (track[next].x - track[idx].x) * frac;
    y = track[idx].y + (track[next].y - track[idx].y) * frac;
}

void Publisher::publishDetections(uint32_t nowMs)
{
    // Mission time runs faster than wall time by the configured scale, which
    // is what makes a mission sampled at ten-second intervals watchable.
    const float missionTimeS =
        (static_cast<float>(nowMs) / 1000.0f) * board::kTimeScale;

    for (int i = 0; i < mission_data::kTrackCount; ++i)
    {
        sensor_link::TargetDetection d{};
        d.t_ms = nowMs;
        d.id   = static_cast<uint8_t>(i);
        sampleTrack(i, missionTimeS, d.x, d.y);

        uart_.send(sensor_link::PKT_TARGET, &d, sizeof d);
    }
}

void Publisher::publishAmmo()
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

void Publisher::publishStatus()
{
    sensor_link::SeekerStatus s{};
    s.trackCount = static_cast<uint8_t>(mission_data::kTrackCount);

    uart_.send(sensor_link::PKT_STATUS, &s, sizeof s);
}

void Publisher::tick(uint32_t nowMs)
{
    // On the first tick everything is due at once, so the receiver learns the
    // store and the track count before the first detections reach it.
    if (!started_)
    {
        started_ = true;
        publishAmmo();
        publishStatus();
        publishDetections(nowMs);
        lastAmmoMs_ = lastStatusMs_ = lastDetectionMs_ = nowMs;
        return;
    }

    if (due(nowMs, lastDetectionMs_, kDetectionPeriodMs))
    {
        lastDetectionMs_ = nowMs;
        publishDetections(nowMs);
    }

    if (due(nowMs, lastAmmoMs_, board::kAmmoRepeatMs))
    {
        lastAmmoMs_ = nowMs;
        publishAmmo();
    }

    if (due(nowMs, lastStatusMs_, board::kStatusRepeatMs))
    {
        lastStatusMs_ = nowMs;
        publishStatus();
    }
}

} // namespace publisher
