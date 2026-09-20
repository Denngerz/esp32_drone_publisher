#include "../../include/mt/SerialTargetSource.hpp"
#include "../../include/link/SensorLink.hpp"
#include "../../include/mt/ConsoleLog.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

SerialTargetSource::SerialTargetSource(std::string device, int baud,
                                       bool expectsAmmo)
    : link_(std::move(device), baud, "Seeker")
    , expectsAmmo_(expectsAmmo)
{
}

bool SerialTargetSource::openPort()
{
    return link_.open();
}

void SerialTargetSource::onDetection(const uint8_t* payload, uint8_t len)
{
    if (len < sizeof(sensor_link::TargetDetection)) return;

    sensor_link::TargetDetection d;
    std::memcpy(&d, payload, sizeof d);

    std::lock_guard<std::mutex> lock(mutex_);

    if (d.id >= tracks_.size())
        tracks_.resize(static_cast<std::size_t>(d.id) + 1);

    Track& t = tracks_[d.id];
    const Coord now{ d.x, d.y };

    // Estimate velocity from the seeker's own clock. Half-and-half smoothing
    // damps the jitter that differencing two noisy detections produces,
    // exactly as the onboard autopilot does with the same data.
    if (t.seen && d.t_ms > t.lastSeenMs)
    {
        const float dt = static_cast<float>(d.t_ms - t.lastSeenMs) / 1000.0f;
        if (dt > 1e-4f)
        {
            const Coord measured = (now - t.pos) / dt;
            t.velocity = t.velocity * 0.5f + measured * 0.5f;
        }
    }

    t.pos         = now;
    t.lastSeenMs  = d.t_ms;
    t.lastArrival = Clock::now();
    t.seen        = true;
}

void SerialTargetSource::onAmmo(const uint8_t* payload, uint8_t len)
{
    if (len < sizeof(sensor_link::AmmoReport)) return;

    sensor_link::AmmoReport a;
    std::memcpy(&a, payload, sizeof a);

    std::lock_guard<std::mutex> lock(mutex_);

    // The name field is not guaranteed to be NUL-terminated on the wire.
    ammo_.name = std::string(a.name, ::strnlen(a.name, sizeof a.name));
    ammo_.mass = a.mass;
    ammo_.drag = a.drag;
    ammo_.lift = a.lift;
    hitRadius_ = a.hitRadius;

    if (!haveAmmo_)
    {
        haveAmmo_ = true;

        std::ostringstream msg;
        msg << "Bay reports " << ammo_.name
            << " mass=" << ammo_.mass
            << " drag=" << ammo_.drag
            << " lift=" << ammo_.lift
            << " hitR=" << hitRadius_;
        console::line(msg.str());
    }
}

void SerialTargetSource::onStatus(const uint8_t* payload, uint8_t len)
{
    if (len < sizeof(sensor_link::SeekerStatus)) return;

    sensor_link::SeekerStatus s;
    std::memcpy(&s, payload, sizeof s);

    std::lock_guard<std::mutex> lock(mutex_);

    if (trackCount_ != s.trackCount)
    {
        trackCount_ = s.trackCount;
        console::line("Seeker holds " + std::to_string(trackCount_) + " track(s)");
    }
    if (tracks_.size() < static_cast<std::size_t>(trackCount_))
        tracks_.resize(static_cast<std::size_t>(trackCount_));
}

void SerialTargetSource::handleFrame(uint8_t type, const uint8_t* payload, uint8_t len)
{
    {
        // Any valid frame proves the link is alive, even one this source has
        // no use for.
        std::lock_guard<std::mutex> lock(mutex_);
        lastFrameArrival_ = Clock::now();
        everReceived_     = true;
    }

    switch (type)
    {
        case sensor_link::PKT_TARGET: onDetection(payload, len); break;
        case sensor_link::PKT_AMMO:   onAmmo(payload, len);      break;
        case sensor_link::PKT_STATUS: onStatus(payload, len);    break;
        default: break;
    }
}

bool SerialTargetSource::waitUntilReady(int timeoutMs)
{
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

    while (std::chrono::steady_clock::now() < deadline)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if ((haveAmmo_ || !expectsAmmo_) && trackCount_ > 0)
                return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::lock_guard<std::mutex> lock(mutex_);
    std::cerr << "Seeker not ready after " << timeoutMs << " ms"
              << " (tracks=" << trackCount_;
    if (expectsAmmo_)
        std::cerr << ", ammo=" << (haveAmmo_ ? "yes" : "no");
    std::cerr << ")\n";
    return false;
}

AmmoParams SerialTargetSource::ammo() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return ammo_;
}

float SerialTargetSource::hitRadius() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return hitRadius_;
}

int SerialTargetSource::getTargetCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return trackCount_;
}

Target SerialTargetSource::getTarget(int index) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (index < 0 || index >= static_cast<int>(tracks_.size()))
        return Target{};

    const Track& t = tracks_[index];

    Target out;
    out.pos = t.pos;

    // Hold the last position, but stop pretending to know where it is going.
    // A stale track carried forward at its old velocity drifts somewhere the
    // seeker never reported, and the mission would aim at that.
    const bool stale = !t.seen || (Clock::now() - t.lastArrival) > kTrackStaleAfter;
    out.velocity = stale ? Coord{ 0.0f, 0.0f } : t.velocity;

    return out;
}

bool SerialTargetSource::healthy() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Before the first frame there is nothing to have lost; waitUntilReady is
    // what covers a link that never speaks at all.
    if (!everReceived_)
        return true;

    return (Clock::now() - lastFrameArrival_) <= kLinkDeadAfter;
}

void SerialTargetSource::start() { started_.store(true); }
void SerialTargetSource::stop()  { link_.stop(); }

void SerialTargetSource::run()
{
    ready_.store(true);

    // Reading begins immediately rather than waiting for start(): the track
    // count has to arrive before the mission can be released, and it only
    // arrives by reading.
    link_.readUntilStopped(
        [this](uint8_t type, const uint8_t* payload, uint8_t len)
        { handleFrame(type, payload, len); });
}
