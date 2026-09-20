#pragma once
// SerialAmmoSource.hpp — the payload bay on a link of its own.
//
// The bay is a separate module from the seeker: a different board, a
// different UART, and a different question answered. It reports what is
// physically loaded, which the flight computer needs before it can solve any
// ballistics at all, and which the mission configuration is not allowed to
// override.
//
// There is deliberately no healthy() here, unlike SerialTargetSource. The two
// links fail differently and the difference is the point. A seeker that goes
// silent has stopped describing a world that keeps moving, so the mission is
// flying blind and must stop. The bay describes something that cannot change
// in flight — the round is in the tube or it is not — so once it has been
// heard, its silence costs nothing. Aborting a mission because a static
// report stopped repeating would be a worse answer than ignoring it.
//
// The reading thread is the only writer; readers take snapshots under the
// same mutex.

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include "../dto/AmmoParams.hpp"
#include "SerialLink.hpp"

class SerialAmmoSource
{
public:
    explicit SerialAmmoSource(std::string device, int baud = 115200);

    SerialAmmoSource(const SerialAmmoSource&)            = delete;
    SerialAmmoSource& operator=(const SerialAmmoSource&) = delete;

    // Opens the port in raw 8N1. False on failure, with the reason on stderr.
    bool openPort();

    // Blocks until the bay has said what it carries, or the timeout expires.
    // The bay repeats itself once a second, so a listener that starts late
    // still catches up well inside a sensible timeout.
    bool waitUntilReady(int timeoutMs);

    AmmoParams ammo() const;
    float      hitRadius() const;

    // --- thread lifecycle ---
    void run();                                        // thread body
    bool isThreadReady() const { return ready_.load(); }
    void stop();

private:
    void handleFrame(uint8_t type, const uint8_t* payload, uint8_t len);
    void onAmmo(const uint8_t* payload, uint8_t len);

    SerialLink link_;

    mutable std::mutex mutex_;
    AmmoParams         ammo_{};
    float              hitRadius_ = 0.0f;
    bool               haveAmmo_  = false;

    std::atomic<bool> ready_{ false };
};
