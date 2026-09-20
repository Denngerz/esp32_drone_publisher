#pragma once
// SerialTargetSource.hpp — targets taken from the ESP32's seeker.
//
// Stands in for ThreadSafeTargetProvider. Where that one replays a trajectory
// file it loaded itself, this one owns no trajectories at all: it holds
// whatever the seeker last reported and nothing more. That is the whole point
// of the substitution — the flight computer stops knowing the future of a
// target and has to work from observations, the way it would on an airframe.
//
// Velocity is not reported by the seeker, so it is estimated here by
// differencing successive detections against the seeker's own clock. Using
// that clock rather than arrival times means a delayed or bunched frame
// produces a correct velocity anyway.
//
// Ammunition may or may not arrive here. With one module publishing
// everything on a single wire it does, and this class reports it. With the
// seeker and the payload bay wired as separate modules, the bay has its own
// link and its own receiver, and this one is told not to wait for a store it
// will never hear about. See SerialAmmoSource.
//
// The reading thread is the only writer; readers take snapshots under the
// same mutex, so nothing hands out a reference into moving state.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "../dto/AmmoParams.hpp"
#include "../dto/Coord.hpp"
#include "../dto/Target.hpp"
#include "../interfaces/ITargetSource.hpp"
#include "SerialLink.hpp"

class SerialTargetSource : public ITargetSource
{
public:
    using Clock = std::chrono::steady_clock;

    // Detections arrive every 50 ms per track at the firmware's default rate,
    // so half a second is ten missed rounds: long enough not to trip on
    // scheduling jitter, short enough that a lost track is noticed within one
    // manoeuvre.
    static constexpr auto kTrackStaleAfter = std::chrono::milliseconds(500);

    // The seeker status repeats once a second, so a second and a half of
    // total silence means nothing is arriving at all, not merely that one
    // track was dropped.
    static constexpr auto kLinkDeadAfter = std::chrono::milliseconds(1500);

    // expectsAmmo is false when the payload bay publishes on a link of its
    // own: this source then becomes ready on the track count alone.
    explicit SerialTargetSource(std::string device, int baud = 115200,
                                bool expectsAmmo = true);

    SerialTargetSource(const SerialTargetSource&)            = delete;
    SerialTargetSource& operator=(const SerialTargetSource&) = delete;

    // Opens the port in raw 8N1. False on failure, with the reason on stderr.
    bool openPort();

    // Blocks until the seeker has reported how many tracks it holds — and,
    // when this link also carries the bay, what is loaded — or until the
    // timeout expires. The mission cannot start before then: a zero track
    // count reads as "nothing left to do".
    bool waitUntilReady(int timeoutMs);

    // What the payload bay reported. Only meaningful when this link carries
    // the bay at all; with a separate bay module, ask SerialAmmoSource.
    AmmoParams ammo() const;
    float      hitRadius() const;

    // --- ITargetSource ---
    int    getTargetCount() const override;

    // A track nobody has confirmed for kTrackStaleAfter is returned at its
    // last known position with zero velocity. Extrapolating a stale
    // observation forward invents a target that was never seen there.
    Target getTarget(int index) const override;

    // False once nothing at all has arrived for kLinkDeadAfter. Reported
    // rather than acted on here: what a mission does about a dead link is
    // the mission's decision.
    bool   healthy() const override;
    void   run() override;
    bool   isThreadReady() const override { return ready_.load(); }
    void   start() override;
    void   stop() override;

private:
    void handleFrame(uint8_t type, const uint8_t* payload, uint8_t len);
    void onDetection(const uint8_t* payload, uint8_t len);
    void onAmmo(const uint8_t* payload, uint8_t len);
    void onStatus(const uint8_t* payload, uint8_t len);

    // One track as the flight computer knows it: the last reported position,
    // an estimated velocity, and the seeker timestamp both were derived from.
    struct Track
    {
        Coord    pos{ 0.0f, 0.0f };
        Coord    velocity{ 0.0f, 0.0f };

        // The seeker's own clock, used for differencing into a velocity.
        uint32_t lastSeenMs = 0;

        // Local arrival time, used only to judge staleness. The two cannot be
        // compared with each other and answer different questions: how fast
        // the target was moving, and whether anyone has looked lately.
        Clock::time_point lastArrival{};

        bool     seen       = false;
    };

    SerialLink link_;
    bool       expectsAmmo_;

    mutable std::mutex mutex_;
    std::vector<Track> tracks_;
    int                trackCount_ = 0;
    AmmoParams         ammo_{};
    float              hitRadius_ = 0.0f;
    bool               haveAmmo_  = false;

    // When anything last arrived, whatever it was. Distinct from per-track
    // arrival: the link can be alive while one track goes unreported.
    Clock::time_point lastFrameArrival_{};
    bool              everReceived_ = false;

    std::atomic<bool> ready_{ false };
    std::atomic<bool> started_{ false };
};
