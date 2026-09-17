// SensorLinkTest.cpp — protocol conformance for include/link/SensorLink.hpp.
//
// The link is the seam between two machines, so the properties worth pinning
// down are the ones a real serial line will exercise: a listener that starts
// late, bytes arriving in arbitrary chunks, and occasional corruption. A
// round trip alone would not catch a parser that wedges after a bad frame.

#include "link/SensorLink.hpp"

#include <cstdio>
#include <cstring>
#include <vector>

using namespace sensor_link;

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

// Feeds a whole buffer through a parser, reporting whether a frame completed.
bool feedAll(Parser& p, const uint8_t* data, size_t n,
             uint8_t& type, uint8_t* payload, uint8_t& len)
{
    bool got = false;
    for (size_t i = 0; i < n; ++i)
        if (p.feed(data[i], type, payload, len))
            got = true;
    return got;
}

void testDetectionRoundTrip()
{
    Parser p;
    uint8_t type = 0, len = 0, payload[260];

    TargetDetection sent{ 1234, 3, 340.5f, -250.25f };
    uint8_t frame[64];
    size_t n = encode(PKT_TARGET, &sent, sizeof sent, frame);

    check(feedAll(p, frame, n, type, payload, len), "detection did not decode");
    check(type == PKT_TARGET, "detection decoded with the wrong type");
    check(len == sizeof sent, "detection decoded with the wrong length");

    TargetDetection got{};
    std::memcpy(&got, payload, sizeof got);
    check(got.t_ms == sent.t_ms && got.id == sent.id &&
          got.x == sent.x && got.y == sent.y, "detection payload changed in transit");
}

void testAmmoRoundTrip()
{
    Parser p;
    uint8_t type = 0, len = 0, payload[260];

    AmmoReport sent{};
    std::memcpy(sent.name, "VOG-17", 6);
    sent.mass = 0.35f; sent.drag = 0.004f; sent.lift = 0.0f; sent.hitRadius = 3.0f;

    uint8_t frame[64];
    size_t n = encode(PKT_AMMO, &sent, sizeof sent, frame);

    check(feedAll(p, frame, n, type, payload, len), "ammo did not decode");
    check(type == PKT_AMMO, "ammo decoded with the wrong type");

    AmmoReport got{};
    std::memcpy(&got, payload, sizeof got);
    check(got.mass == sent.mass && got.drag == sent.drag &&
          got.hitRadius == sent.hitRadius, "ammo payload changed in transit");
    check(std::memcmp(got.name, sent.name, 6) == 0, "ammo name changed in transit");
}

// The flight computer may be started after the seeker is already talking, so
// the parser has to find a frame boundary in a stream it joined mid-way.
// The noise deliberately contains stray magic bytes.
void testResyncFromMidStream()
{
    Parser p;
    uint8_t type = 0, len = 0, payload[260];

    const std::vector<uint8_t> noise{ 0x11, 0xA5, 0x22, 0xA5, 0xA5, 0x00, 0xFF };
    for (uint8_t b : noise)
        p.feed(b, type, payload, len);

    SeekerStatus sent{ 5 };
    uint8_t frame[64];
    size_t n = encode(PKT_STATUS, &sent, sizeof sent, frame);

    check(feedAll(p, frame, n, type, payload, len), "did not resync after garbage");
    check(type == PKT_STATUS && payload[0] == 5, "resynced onto the wrong frame");
}

// A corrupt frame must be dropped, and — the part that matters — must not
// take the following good frame down with it.
void testCorruptFrameIsRejectedAndRecovered()
{
    Parser p;
    uint8_t type = 0, len = 0, payload[260];

    TargetDetection sent{ 42, 1, 10.0f, 20.0f };
    uint8_t frame[64];
    size_t n = encode(PKT_TARGET, &sent, sizeof sent, frame);

    frame[6] ^= 0xFF;   // flip a payload bit, leaving the CRC stale
    check(!feedAll(p, frame, n, type, payload, len), "corrupt frame was accepted");

    n = encode(PKT_TARGET, &sent, sizeof sent, frame);
    check(feedAll(p, frame, n, type, payload, len),
          "parser did not recover after a corrupt frame");
}

// UART reads return whatever happens to be buffered, so a frame is routinely
// split across two of them.
void testFrameSplitAcrossReads()
{
    Parser p;
    uint8_t type = 0, len = 0, payload[260];

    TargetDetection sent{ 7, 2, -1.5f, 2.5f };
    uint8_t frame[64];
    size_t n = encode(PKT_TARGET, &sent, sizeof sent, frame);

    feedAll(p, frame, 3, type, payload, len);
    check(feedAll(p, frame + 3, n - 3, type, payload, len),
          "frame split across reads did not assemble");
}

} // namespace

int main()
{
    testDetectionRoundTrip();
    testAmmoRoundTrip();
    testResyncFromMidStream();
    testCorruptFrameIsRejectedAndRecovered();
    testFrameSplitAcrossReads();

    if (failures == 0)
        std::printf("all protocol checks passed\n");
    else
        std::printf("%d check(s) failed\n", failures);

    return failures == 0 ? 0 : 1;
}
