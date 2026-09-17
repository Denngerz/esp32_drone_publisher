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

// xorshift32: a handful of instructions, no library, and reproducible across
// runs and builds. Nothing here needs cryptographic quality, only a spread.
float Publisher::nextUniform()
{
    rngState_ ^= rngState_ << 13;
    rngState_ ^= rngState_ >> 17;
    rngState_ ^= rngState_ << 5;
    return static_cast<float>(rngState_) / 4294967296.0f;   // [0, 1)
}

// Box-Muller. Both halves of the pair would be usable, but discarding one
// keeps the call site simple and the cost is irrelevant at a hundred samples
// a second.
float Publisher::nextNormal()
{
    // Guard the log against a zero draw, which xorshift32 can produce.
    const float u1 = nextUniform() + 1e-7f;
    const float u2 = nextUniform();
    return std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * 3.14159265f * u2);
}

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
        // A track already dropped stays silent until its hold expires. The
        // receiver sees this as a track that stopped being mentioned, which
        // is exactly what a masked target looks like from the air.
        if (droppedUntilMs_[i] != 0)
        {
            if (static_cast<int32_t>(nowMs - droppedUntilMs_[i]) < 0)
                continue;
            droppedUntilMs_[i] = 0;
        }
        else if (board::kTrackDropChance > 0.0f &&
                 nextUniform() < board::kTrackDropChance)
        {
            droppedUntilMs_[i] = nowMs + board::kTrackDropHoldMs;
            continue;
        }

        float x = 0.0f, y = 0.0f;
        sampleTrack(i, missionTimeS, x, y);

        // Measurement error, independent on each axis.
        if (board::kDetectionNoiseM > 0.0f)
        {
            x += nextNormal() * board::kDetectionNoiseM;
            y += nextNormal() * board::kDetectionNoiseM;
        }

        sensor_link::TargetDetection d{};
        d.t_ms = nowMs;
        d.id   = static_cast<uint8_t>(i);
        d.x    = x;
        d.y    = y;

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
