#pragma once
// seeker.hpp — the seeker module.
//
// It holds one track per target and walks each along its baked trajectory,
// reporting an interpolated position at a fixed rate. Each detection carries
// the seeker's own clock, so the receiver can difference two of them into a
// velocity without trusting when the frames arrived. It also repeats how many
// tracks it holds: the receiver cannot infer that from detections alone
// without waiting to see every id.
//
// Neither the positions nor the track list are perfect. Each reported
// position carries a measurement error, and a track occasionally drops out
// for a while, as a real seeker's would when a target is masked or the
// tracker loses lock. Without that the receiver's smoothing and staleness
// handling would never be exercised by anything.
//
// The payload bay is a separate module on a separate link; see bay.hpp.
// Nothing is shared between the two but the chip.
//
// Timing is passed in rather than read from a clock inside, which keeps the
// whole class testable on a host and free of FreeRTOS.

#include "mission_data.generated.hpp"

#include <cstdint>

namespace sensor_uart { class Uart; }

namespace seeker
{

class Seeker
{
public:
    explicit Seeker(sensor_uart::Uart& uart) : uart_(uart) {}

    // Call often; it decides internally what is due. nowMs is wall-clock
    // milliseconds since boot.
    void tick(uint32_t nowMs);

private:
    void publishDetections(uint32_t nowMs);
    void publishStatus();

    // Deterministic noise. A fixed seed means two runs of the same firmware
    // produce the same errors, which is what makes a disagreement between
    // runs mean something.
    float nextUniform();
    float nextNormal();

    // Interpolated position of one track at a given mission time. Tracks are
    // cyclic: a mission longer than the baked trajectory wraps to the start
    // rather than running off the end.
    void sampleTrack(int index, float missionTimeS, float& x, float& y) const;

    sensor_uart::Uart& uart_;

    // Per track: the time until which it stays unreported. Zero means it is
    // being reported normally.
    uint32_t droppedUntilMs_[mission_data::kTrackCount] = {};

    uint32_t rngState_ = 0x9E3779B9u;

    uint32_t lastDetectionMs_ = 0;
    uint32_t lastStatusMs_    = 0;
    bool     started_         = false;
};

} // namespace seeker
