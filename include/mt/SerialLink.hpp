#pragma once
// SerialLink.hpp — one serial connection to one sensor module.
//
// The flight computer listens to more than one module: the seeker reports
// what it sees, the payload bay reports what is loaded, and on the aircraft
// they are separate boxes on separate wires. Everything below the frame
// boundary is identical for both — open the port raw, read whatever bytes
// turned up, feed them to the parser — so it lives here once and the sources
// above only decide what a completed frame means.
//
// read() returns on a timer rather than blocking forever, so a link that has
// gone quiet still notices stop() instead of hanging its thread.

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

class SerialLink
{
public:
    // Called for each frame whose CRC checked out.
    using FrameSink =
        std::function<void(uint8_t type, const uint8_t* payload, uint8_t len)>;

    // label names the module in log lines: "Seeker", "Bay".
    SerialLink(std::string device, int baud, std::string label);
    ~SerialLink();

    SerialLink(const SerialLink&)            = delete;
    SerialLink& operator=(const SerialLink&) = delete;

    // Opens the port in raw 8N1. False on failure, with the reason on stderr.
    bool open();

    // Thread body: reads until stop(), handing each valid frame to onFrame.
    void readUntilStopped(const FrameSink& onFrame);

    void stop();
    bool running() const { return running_.load(); }

    const std::string& device() const { return device_; }

private:
    std::string       device_;
    int               baud_;
    std::string       label_;
    int               fd_ = -1;
    std::atomic<bool> running_{ true };
};
