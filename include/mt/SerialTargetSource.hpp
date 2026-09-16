#pragma once
// SerialTargetSource.hpp — targets and ammunition taken from the ESP32.
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
// The reading thread is the only writer; readers take snapshots under the
// same mutex, so nothing hands out a reference into moving state.

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "../dto/AmmoParams.hpp"
#include "../dto/Coord.hpp"
#include "../dto/Target.hpp"
#include "../interfaces/ITargetSource.hpp"

class SerialTargetSource : public ITargetSource
{
public:
    explicit SerialTargetSource(std::string device, int baud = 115200);
    ~SerialTargetSource() override;

    SerialTargetSource(const SerialTargetSource&)            = delete;
    SerialTargetSource& operator=(const SerialTargetSource&) = delete;

    // Opens the port in raw 8N1. False on failure, with the reason on stderr.
    bool openPort();

    // Blocks until the seeker has reported both how many tracks it holds and
    // what the bay carries, or until the timeout expires. The mission cannot
    // start before then: a zero track count reads as "nothing left to do",
    // and the ballistics need the store's properties.
    bool waitUntilReady(int timeoutMs);

    // What the payload bay reported. Only meaningful after waitUntilReady.
    AmmoParams ammo() const;
    float      hitRadius() const;

    // --- ITargetSource ---
    int    getTargetCount() const override;
    Target getTarget(int index) const override;
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
        uint32_t lastSeenMs = 0;
        bool     seen       = false;
    };

    std::string device_;
    int         baud_;
    int         fd_ = -1;

    mutable std::mutex mutex_;
    std::vector<Track> tracks_;
    int                trackCount_ = 0;
    AmmoParams         ammo_{};
    float              hitRadius_ = 0.0f;
    bool               haveAmmo_  = false;

    std::atomic<bool> ready_{ false };
    std::atomic<bool> started_{ false };
    std::atomic<bool> running_{ true };
};
