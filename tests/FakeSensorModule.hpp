#pragma once
// FakeSensorModule.hpp — the stand-in both receiver suites run against, plus
// the handful of assertions they share.
//
// The module is a pseudo-terminal, so the receiver under test opens a real
// character device and runs its real reading thread. Only the far end is
// synthetic. It can play either role: a seeker sending detections and status,
// or a payload bay sending a store report, because on the wire the two differ
// only in which packet types they emit.

#include "link/SensorLink.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include <pty.h>
#include <termios.h>
#include <unistd.h>

namespace testing
{

inline int failures = 0;

inline void check(bool ok, const char* what)
{
    if (!ok)
    {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

inline void checkNear(float got, float want, float tol, const char* what)
{
    const float diff = got > want ? got - want : want - got;
    if (diff > tol)
    {
        std::printf("FAIL: %s (got %.3f, wanted %.3f +/- %.3f)\n", what, got, want, tol);
        ++failures;
    }
}

inline void sleepMs(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline int report(const char* suite)
{
    if (failures == 0)
        std::printf("all %s checks passed\n", suite);
    else
        std::printf("%d check(s) failed\n", failures);

    return failures == 0 ? 0 : 1;
}

// One end of a pseudo-terminal pair, standing in for a module on the ESP32.
class FakeModule
{
public:
    bool open()
    {
        if (::openpty(&master_, &slave_, nullptr, nullptr, nullptr) != 0)
            return false;

        // Raw mode: the line discipline would otherwise rewrite bytes that
        // happen to look like control characters and corrupt binary frames.
        termios tio{};
        ::tcgetattr(slave_, &tio);
        ::cfmakeraw(&tio);
        ::tcsetattr(slave_, TCSANOW, &tio);

        path_ = ::ttyname(slave_);
        return !path_.empty();
    }

    void close()
    {
        if (master_ >= 0) { ::close(master_); master_ = -1; }
        if (slave_  >= 0) { ::close(slave_);  slave_  = -1; }
    }

    const std::string& path() const { return path_; }

    void sendAmmo(const char* name, float mass, float drag, float lift, float hitR)
    {
        sensor_link::AmmoReport a{};
        std::memcpy(a.name, name, std::strlen(name) < sizeof a.name
                                      ? std::strlen(name) : sizeof a.name);
        a.mass = mass; a.drag = drag; a.lift = lift; a.hitRadius = hitR;
        send(sensor_link::PKT_AMMO, &a, sizeof a);
    }

    void sendStatus(uint8_t trackCount)
    {
        sensor_link::SeekerStatus s{ trackCount };
        send(sensor_link::PKT_STATUS, &s, sizeof s);
    }

    void sendDetection(uint32_t tMs, uint8_t id, float x, float y)
    {
        sensor_link::TargetDetection d{ tMs, id, x, y };
        send(sensor_link::PKT_TARGET, &d, sizeof d);
    }

private:
    void send(uint8_t type, const void* payload, uint8_t len)
    {
        uint8_t frame[280];
        const size_t n = sensor_link::encode(type, payload, len, frame);
        const ssize_t written = ::write(master_, frame, n);
        (void)written;
    }

    int         master_ = -1;
    int         slave_  = -1;
    std::string path_;
};

} // namespace testing
