#include "../../include/mt/SerialAmmoSource.hpp"
#include "../../include/link/SensorLink.hpp"
#include "../../include/mt/ConsoleLog.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>
#include <thread>

SerialAmmoSource::SerialAmmoSource(std::string device, int baud)
    : link_(std::move(device), baud, "Bay")
{
}

bool SerialAmmoSource::openPort()
{
    return link_.open();
}

void SerialAmmoSource::onAmmo(const uint8_t* payload, uint8_t len)
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

void SerialAmmoSource::handleFrame(uint8_t type, const uint8_t* payload, uint8_t len)
{
    // Detections and seeker status have no business on this link. If they do
    // turn up — a wire on the wrong header pin, both modules talking to the
    // same port — they are ignored rather than acted on, so a miswiring shows
    // up as a bay that never reports instead of as a mission flying on
    // targets from a link that was supposed to carry none.
    if (type == sensor_link::PKT_AMMO)
        onAmmo(payload, len);
}

bool SerialAmmoSource::waitUntilReady(int timeoutMs)
{
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

    while (std::chrono::steady_clock::now() < deadline)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (haveAmmo_)
                return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cerr << "Bay said nothing in " << timeoutMs << " ms\n";
    return false;
}

AmmoParams SerialAmmoSource::ammo() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return ammo_;
}

float SerialAmmoSource::hitRadius() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return hitRadius_;
}

void SerialAmmoSource::stop() { link_.stop(); }

void SerialAmmoSource::run()
{
    ready_.store(true);

    link_.readUntilStopped(
        [this](uint8_t type, const uint8_t* payload, uint8_t len)
        { handleFrame(type, payload, len); });
}
