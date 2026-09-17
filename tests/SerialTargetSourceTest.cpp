// SerialTargetSourceTest.cpp — the receiving half of the sensor link.
//
// SerialTargetSource is the piece that turns a byte stream into something the
// mission can fly against, and most of what it does cannot be read off the
// code: a velocity nobody reports has to be inferred, and a track nobody
// mentions any more has to stop looking alive. Those are the properties
// pinned down here.
//
// The link is a pseudo-terminal, so the source opens a real character device
// and runs its real reading thread. Only the far end is synthetic.

#include "link/SensorLink.hpp"
#include "mt/SerialTargetSource.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

#include <pty.h>
#include <termios.h>
#include <unistd.h>

namespace
{

int failures = 0;

void check(bool ok, const char* what)
{
    if (!ok)
    {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

void checkNear(float got, float want, float tol, const char* what)
{
    const float diff = got > want ? got - want : want - got;
    if (diff > tol)
    {
        std::printf("FAIL: %s (got %.3f, wanted %.3f +/- %.3f)\n", what, got, want, tol);
        ++failures;
    }
}

void sleepMs(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// One end of a pseudo-terminal pair, standing in for the ESP32.
class FakeSeeker
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

// Brings up a seeker and a source already reading from it.
struct Rig
{
    FakeSeeker         seeker;
    SerialTargetSource source;
    std::thread        thread;
    bool               ok = false;

    explicit Rig(FakeSeeker& s) : source(s.path()) {}

    void start()
    {
        ok = source.openPort();
        if (!ok) return;
        thread = std::thread(&SerialTargetSource::run, &source);
        while (!source.isThreadReady())
            sleepMs(1);
    }

    ~Rig()
    {
        source.stop();
        if (thread.joinable()) thread.join();
    }
};

// The mission cannot start until the bay and the track count are known, so
// that handshake is the first thing worth proving.
void testLearnsAmmoAndTrackCount()
{
    FakeSeeker seeker;
    if (!seeker.open()) { check(false, "could not open a pseudo-terminal"); return; }

    Rig rig(seeker);
    rig.start();
    if (!rig.ok) { check(false, "source could not open the pty"); seeker.close(); return; }

    seeker.sendAmmo("VOG-17", 0.35f, 0.004f, 0.0f, 3.0f);
    seeker.sendStatus(5);

    check(rig.source.waitUntilReady(2000), "waitUntilReady timed out");
    check(rig.source.getTargetCount() == 5, "track count was not learned");

    const AmmoParams a = rig.source.ammo();
    check(a.name == "VOG-17", "ammo name was not learned");
    checkNear(a.mass, 0.35f, 1e-6f, "ammo mass");
    checkNear(rig.source.hitRadius(), 3.0f, 1e-6f, "hit radius");

    seeker.close();
}

// The seeker reports position only. A velocity the mission can lead with has
// to be inferred from two detections against the seeker's own clock.
void testEstimatesVelocityFromTwoDetections()
{
    FakeSeeker seeker;
    if (!seeker.open()) { check(false, "could not open a pseudo-terminal"); return; }

    Rig rig(seeker);
    rig.start();
    if (!rig.ok) { check(false, "source could not open the pty"); seeker.close(); return; }

    seeker.sendAmmo("VOG-17", 0.35f, 0.004f, 0.0f, 3.0f);
    seeker.sendStatus(1);
    rig.source.waitUntilReady(2000);

    // Two seconds apart on the seeker's clock, twenty metres along x.
    // The estimator is half-and-half smoothed from a standing start, so one
    // step reaches half of the true rate.
    seeker.sendDetection(1000, 0, 100.0f, 200.0f);
    sleepMs(50);
    seeker.sendDetection(3000, 0, 120.0f, 200.0f);
    sleepMs(50);

    Target t = rig.source.getTarget(0);
    checkNear(t.pos.x, 120.0f, 0.01f, "position x after two detections");
    checkNear(t.pos.y, 200.0f, 0.01f, "position y after two detections");
    checkNear(t.velocity.x, 5.0f, 0.01f, "velocity x, half of 10 m/s after one step");
    checkNear(t.velocity.y, 0.0f, 0.01f, "velocity y should stay zero");

    seeker.close();
}

// A track nobody confirms any more must stop looking like it is still moving.
// Carrying the old velocity forward would aim the mission at a position the
// seeker never reported.
void testStaleTrackLosesItsVelocity()
{
    FakeSeeker seeker;
    if (!seeker.open()) { check(false, "could not open a pseudo-terminal"); return; }

    Rig rig(seeker);
    rig.start();
    if (!rig.ok) { check(false, "source could not open the pty"); seeker.close(); return; }

    seeker.sendAmmo("VOG-17", 0.35f, 0.004f, 0.0f, 3.0f);
    seeker.sendStatus(1);
    rig.source.waitUntilReady(2000);

    seeker.sendDetection(1000, 0, 100.0f, 200.0f);
    sleepMs(50);
    seeker.sendDetection(3000, 0, 120.0f, 200.0f);
    sleepMs(50);

    check(rig.source.getTarget(0).velocity.x != 0.0f, "velocity should be non-zero while fresh");

    // Keep the link alive with status frames while saying nothing about the
    // track, so this isolates track staleness from link death.
    for (int i = 0; i < 8; ++i) { seeker.sendStatus(1); sleepMs(100); }

    Target t = rig.source.getTarget(0);
    checkNear(t.pos.x, 120.0f, 0.01f, "stale track should hold its last position");
    checkNear(t.velocity.x, 0.0f, 1e-6f, "stale track should report zero velocity");
    check(rig.source.healthy(), "link should still be healthy, only the track went stale");

    seeker.close();
}

// Total silence is a different failure from one lost track, and the mission
// needs to be told about it.
void testLinkGoesUnhealthyOnSilence()
{
    FakeSeeker seeker;
    if (!seeker.open()) { check(false, "could not open a pseudo-terminal"); return; }

    Rig rig(seeker);
    rig.start();
    if (!rig.ok) { check(false, "source could not open the pty"); seeker.close(); return; }

    seeker.sendAmmo("VOG-17", 0.35f, 0.004f, 0.0f, 3.0f);
    seeker.sendStatus(2);
    rig.source.waitUntilReady(2000);

    check(rig.source.healthy(), "should be healthy right after a frame");

    sleepMs(2000);   // longer than kLinkDeadAfter
    check(!rig.source.healthy(), "should report unhealthy after prolonged silence");

    seeker.close();
}

} // namespace

int main()
{
    testLearnsAmmoAndTrackCount();
    testEstimatesVelocityFromTwoDetections();
    testStaleTrackLosesItsVelocity();
    testLinkGoesUnhealthyOnSilence();

    if (failures == 0)
        std::printf("all serial source checks passed\n");
    else
        std::printf("%d check(s) failed\n", failures);

    return failures == 0 ? 0 : 1;
}
