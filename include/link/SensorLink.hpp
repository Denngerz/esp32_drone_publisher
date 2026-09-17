#pragma once
// SensorLink.hpp — binary UART protocol between the ESP32 sensor module and
// the Raspberry Pi flight computer.
//
// Roles. The ESP32 is a peripheral that reports what it observes and what it
// carries; it never receives commands. It publishes:
//   * PKT_AMMO   — the payload bay identifying the loaded store, once at
//                  start and then occasionally, so a late listener catches up;
//   * PKT_TARGET — one seeker detection, streamed as targets move;
//   * PKT_STATUS — how many tracks the seeker currently holds.
// The Pi consumes these in place of reading data/targets.json and the ammo
// entry out of data/ammo.json.
//
// Framing follows the course's drone_link design, which is proven and worth
// keeping: a self-synchronising frame with a CRC, so a listener that joins
// mid-stream or drops bytes resynchronises on its own.
//
//   [0]  MAGIC0 = 0xA5
//   [1]  MAGIC1 = 0x5A
//   [2]  TYPE                     packet type
//   [3]  LEN                      payload length in bytes
//   [4..4+LEN-1]  payload         little-endian
//   [..] CRC16 (2 bytes, LE)      CRC-16/CCITT-FALSE over TYPE+LEN+payload
//
// Both ends are little-endian with IEEE-754 floats (Xtensa LX6 and aarch64),
// and every payload is packed, so structs go on the wire as they sit in
// memory. That assumption is what makes the memcpy decode below legitimate;
// it would have to change for a big-endian peer.

#ifndef SENSOR_LINK_HPP
#define SENSOR_LINK_HPP

#include <cstdint>
#include <cstring>

namespace sensor_link
{

constexpr uint8_t MAGIC0 = 0xA5;
constexpr uint8_t MAGIC1 = 0x5A;

enum PacketType : uint8_t
{
    PKT_AMMO   = 0x01,  // payload bay -> flight computer
    PKT_TARGET = 0x02,  // seeker      -> flight computer
    PKT_STATUS = 0x03,  // seeker      -> flight computer
};

#pragma pack(push, 1)

// What the payload bay is carrying. The flight computer needs these to solve
// the ballistics; it no longer looks them up in a local file.
struct AmmoReport
{
    char  name[16];    // e.g. "VOG-17", not necessarily NUL-terminated
    float mass;        // kg
    float drag;        // drag coefficient
    float lift;        // lift coefficient
    float hitRadius;   // metres, what counts as a hit
};

// One seeker detection.
//
// t_ms is the seeker's own clock, not the flight computer's. It is what lets
// the receiver difference two detections into a velocity without trusting the
// link's timing: frames can be delayed or bunched, but the timestamps inside
// them stay true to when the target was actually seen.
struct TargetDetection
{
    uint32_t t_ms;     // seeker time since boot, milliseconds
    uint8_t  id;       // track id, stable for the life of the track
    float    x, y;     // position in the mission plane, metres
};

// How many tracks the seeker holds. The flight computer needs the count up
// front to know when a mission is finished, and cannot infer it from ids
// alone without waiting to see every one of them.
struct SeekerStatus
{
    uint8_t trackCount;
};

#pragma pack(pop)

// ---- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) ----
inline uint16_t crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

// Packs one frame into out, which must hold at least 6 + payloadLen bytes.
// Returns the number of bytes written.
inline size_t encode(uint8_t type, const void* payload, uint8_t payloadLen,
                     uint8_t* out)
{
    out[0] = MAGIC0;
    out[1] = MAGIC1;
    out[2] = type;
    out[3] = payloadLen;
    if (payloadLen && payload)
        std::memcpy(out + 4, payload, payloadLen);

    uint16_t c = crc16(out + 2, static_cast<size_t>(payloadLen) + 2);
    out[4 + payloadLen]     = static_cast<uint8_t>(c & 0xFF);
    out[4 + payloadLen + 1] = static_cast<uint8_t>(c >> 8);

    return static_cast<size_t>(payloadLen) + 6;
}

// Incremental parser: feed it bytes as they arrive. Returns true on a
// complete frame whose CRC checks out, leaving the result in the out
// parameters. Holds its state between calls, so a frame split across several
// reads is assembled correctly and a corrupt one is dropped without taking
// the next frame down with it.
struct Parser
{
    enum State { S_M0, S_M1, S_TYPE, S_LEN, S_PAYLOAD, S_CRC0, S_CRC1 };

    State    st  = S_M0;
    uint8_t  type = 0;
    uint8_t  len  = 0;
    uint8_t  idx  = 0;
    uint8_t  buf[260] = {};
    uint16_t crcRx = 0;

    bool feed(uint8_t byte, uint8_t& outType, uint8_t* outPayload, uint8_t& outLen)
    {
        switch (st)
        {
        case S_M0:
            if (byte == MAGIC0) st = S_M1;
            break;

        case S_M1:
            // A byte that is not MAGIC1 may itself be the start of a real
            // frame, so fall back to S_M1 rather than S_M0 when it is MAGIC0.
            st = (byte == MAGIC1) ? S_TYPE : (byte == MAGIC0 ? S_M1 : S_M0);
            break;

        case S_TYPE:
            type = byte;
            st   = S_LEN;
            break;

        case S_LEN:
            len = byte;
            idx = 0;
            st  = len ? S_PAYLOAD : S_CRC0;
            break;

        case S_PAYLOAD:
            buf[idx++] = byte;
            if (idx >= len) st = S_CRC0;
            break;

        case S_CRC0:
            crcRx = byte;
            st    = S_CRC1;
            break;

        case S_CRC1:
        {
            crcRx |= static_cast<uint16_t>(byte) << 8;
            st = S_M0;

            uint8_t tmp[262];
            tmp[0] = type;
            tmp[1] = len;
            std::memcpy(tmp + 2, buf, len);

            if (crc16(tmp, static_cast<size_t>(len) + 2) == crcRx)
            {
                outType = type;
                outLen  = len;
                std::memcpy(outPayload, buf, len);
                return true;
            }
            break;   // bad CRC: drop the frame and resynchronise
        }
        }
        return false;
    }
};

} // namespace sensor_link

#endif // SENSOR_LINK_HPP
