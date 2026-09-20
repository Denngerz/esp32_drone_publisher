// SerialAmmoSourceTest.cpp — the receiving half of the payload bay's link.
//
// The bay's receiver is far simpler than the seeker's, and the properties
// worth pinning down are about what it refuses to do rather than what it
// computes: it must learn the store from a link of its own, and it must not
// mistake traffic that has no business being there for a store report. A
// miswired header pin is the failure this guards against, and it should
// surface as a bay that never reports rather than as a mission that flew on
// something it misread.

#include "FakeSensorModule.hpp"
#include "mt/SerialAmmoSource.hpp"

#include <thread>

using testing::check;
using testing::checkNear;
using testing::FakeModule;
using testing::sleepMs;

namespace
{

// Brings up a bay and a receiver already reading from it.
struct BayRig
{
    SerialAmmoSource source;
    std::thread      thread;
    bool             ok = false;

    explicit BayRig(FakeModule& module) : source(module.path()) {}

    void start()
    {
        ok = source.openPort();
        if (!ok) return;
        thread = std::thread(&SerialAmmoSource::run, &source);
        while (!source.isThreadReady())
            sleepMs(1);
    }

    ~BayRig()
    {
        source.stop();
        if (thread.joinable()) thread.join();
    }
};

// The ballistics cannot be solved without the store, so this is the whole
// reason the bay's link exists.
void testLearnsStoreFromItsOwnLink()
{
    FakeModule bay;
    if (!bay.open()) { check(false, "could not open a pseudo-terminal"); return; }

    BayRig rig(bay);
    rig.start();
    if (!rig.ok) { check(false, "source could not open the pty"); bay.close(); return; }

    bay.sendAmmo("RKG-3", 1.2f, 0.007f, 0.0f, 4.5f);

    check(rig.source.waitUntilReady(2000), "bay link timed out");

    const AmmoParams a = rig.source.ammo();
    check(a.name == "RKG-3", "store name was not learned");
    checkNear(a.mass, 1.2f, 1e-6f, "store mass");
    checkNear(a.drag, 0.007f, 1e-6f, "store drag");
    checkNear(rig.source.hitRadius(), 4.5f, 1e-6f, "hit radius");

    bay.close();
}

// The bay repeats itself so a listener that starts late still catches up.
// Nothing about a later report should disturb what was already learned.
void testRepeatedReportsAreConsistent()
{
    FakeModule bay;
    if (!bay.open()) { check(false, "could not open a pseudo-terminal"); return; }

    BayRig rig(bay);
    rig.start();
    if (!rig.ok) { check(false, "source could not open the pty"); bay.close(); return; }

    for (int i = 0; i < 4; ++i) { bay.sendAmmo("M67", 0.6f, 0.005f, 0.0f, 2.0f); sleepMs(30); }

    check(rig.source.waitUntilReady(2000), "bay link timed out");
    check(rig.source.ammo().name == "M67", "store name after repeated reports");
    checkNear(rig.source.hitRadius(), 2.0f, 1e-6f, "hit radius after repeated reports");

    bay.close();
}

// Detections and seeker status on this wire mean the modules are crossed.
// The bay's receiver has no use for either and must stay unready rather than
// letting a mission start on a store it never heard about.
void testIgnoresSeekerTraffic()
{
    FakeModule bay;
    if (!bay.open()) { check(false, "could not open a pseudo-terminal"); return; }

    BayRig rig(bay);
    rig.start();
    if (!rig.ok) { check(false, "source could not open the pty"); bay.close(); return; }

    bay.sendDetection(1000, 0, 100.0f, 200.0f);
    bay.sendStatus(4);
    sleepMs(100);

    // The "Bay said nothing" line this prints is the expected outcome, not a
    // failure: a short timeout is how the check is made.
    check(!rig.source.waitUntilReady(200), "seeker traffic must not make the bay ready");

    // And the wrong traffic must not have wedged anything: a real report that
    // arrives afterwards is still heard.
    bay.sendAmmo("M67", 0.6f, 0.005f, 0.0f, 2.0f);
    check(rig.source.waitUntilReady(2000), "a real store report should still be heard");
    check(rig.source.ammo().name == "M67", "store name after the wrong traffic");

    bay.close();
}

} // namespace

int main()
{
    testLearnsStoreFromItsOwnLink();
    testRepeatedReportsAreConsistent();
    testIgnoresSeekerTraffic();

    return testing::report("bay source");
}
