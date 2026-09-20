// SerialTargetSourceTest.cpp — the receiving half of the seeker's link.
//
// SerialTargetSource is the piece that turns a byte stream into something the
// mission can fly against, and most of what it does cannot be read off the
// code: a velocity nobody reports has to be inferred, and a track nobody
// mentions any more has to stop looking alive. Those are the properties
// pinned down here.
//
// The link is a pseudo-terminal, so the source opens a real character device
// and runs its real reading thread. Only the far end is synthetic. See
// FakeSensorModule.hpp; the bay's receiver has its own suite next door.

#include "FakeSensorModule.hpp"
#include "mt/SerialTargetSource.hpp"

#include <thread>

using testing::check;
using testing::checkNear;
using testing::FakeModule;
using testing::sleepMs;

namespace
{

// Brings up a seeker and a source already reading from it.
struct Rig
{
    SerialTargetSource source;
    std::thread        thread;
    bool               ok = false;

    // expectsAmmo mirrors the flag main passes: false when the payload bay is
    // a module of its own and this link carries no store report.
    explicit Rig(FakeModule& module, bool expectsAmmo = true)
        : source(module.path(), 115200, expectsAmmo) {}

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
    FakeModule seeker;
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

// With the bay wired as its own module, no store report will ever arrive on
// this link. Waiting for one would hang the mission before it started, so the
// track count alone has to be enough.
void testReadyWithoutAmmoWhenBayIsSeparate()
{
    FakeModule seeker;
    if (!seeker.open()) { check(false, "could not open a pseudo-terminal"); return; }

    Rig rig(seeker, /*expectsAmmo=*/false);
    rig.start();
    if (!rig.ok) { check(false, "source could not open the pty"); seeker.close(); return; }

    seeker.sendStatus(3);

    check(rig.source.waitUntilReady(2000),
          "should be ready on the track count alone when the bay is separate");
    check(rig.source.getTargetCount() == 3, "track count was not learned");

    seeker.close();
}

// The seeker reports position only. A velocity the mission can lead with has
// to be inferred from two detections against the seeker's own clock.
void testEstimatesVelocityFromTwoDetections()
{
    FakeModule seeker;
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
    FakeModule seeker;
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
    FakeModule seeker;
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
    testReadyWithoutAmmoWhenBayIsSeparate();
    testEstimatesVelocityFromTwoDetections();
    testStaleTrackLosesItsVelocity();
    testLinkGoesUnhealthyOnSilence();

    return testing::report("serial source");
}
