#include "../../include/mt/SerialTargetSource.hpp"
#include "../../include/link/SensorLink.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace
{
// Maps a baud rate to its termios constant. Only the rates this link uses are
// listed; anything else is refused rather than silently run at the wrong speed.
bool toSpeed(int baud, speed_t& out)
{
    switch (baud)
    {
        case 9600:   out = B9600;   return true;
        case 19200:  out = B19200;  return true;
        case 38400:  out = B38400;  return true;
        case 57600:  out = B57600;  return true;
        case 115200: out = B115200; return true;
        case 230400: out = B230400; return true;
        default:                    return false;
    }
}
} // namespace

SerialTargetSource::SerialTargetSource(std::string device, int baud)
    : device_(std::move(device)), baud_(baud)
{
}

SerialTargetSource::~SerialTargetSource()
{
    if (fd_ >= 0)
        ::close(fd_);
}

bool SerialTargetSource::openPort()
{
    speed_t speed;
    if (!toSpeed(baud_, speed))
    {
        std::cerr << "Unsupported baud rate: " << baud_ << '\n';
        return false;
    }

    fd_ = ::open(device_.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0)
    {
        std::cerr << "Cannot open " << device_ << ": " << std::strerror(errno) << '\n';
        return false;
    }

    termios tio{};
    if (::tcgetattr(fd_, &tio) != 0)
    {
        std::cerr << "tcgetattr failed on " << device_ << '\n';
        return false;
    }

    ::cfmakeraw(&tio);                 // 8N1, no character processing
    ::cfsetispeed(&tio, speed);
    ::cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);

    // Return from read() after a tenth of a second even with nothing pending,
    // so the thread notices stop() instead of blocking on a silent link.
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1;

    if (::tcsetattr(fd_, TCSANOW, &tio) != 0)
    {
        std::cerr << "tcsetattr failed on " << device_ << '\n';
        return false;
    }

    ::tcflush(fd_, TCIFLUSH);   // drop whatever arrived before we were listening

    std::cout << "Seeker link open on " << device_ << " @ " << baud_ << " baud\n";
    return true;
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

    t.pos        = now;
    t.lastSeenMs = d.t_ms;
    t.seen       = true;
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
        std::cout << "Bay reports " << ammo_.name
                  << " mass=" << ammo_.mass
                  << " drag=" << ammo_.drag
                  << " lift=" << ammo_.lift
                  << " hitR=" << hitRadius_ << '\n';
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
        std::cout << "Seeker holds " << trackCount_ << " track(s)\n";
    }
    if (tracks_.size() < static_cast<std::size_t>(trackCount_))
        tracks_.resize(static_cast<std::size_t>(trackCount_));
}

void SerialTargetSource::handleFrame(uint8_t type, const uint8_t* payload, uint8_t len)
{
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
            if (haveAmmo_ && trackCount_ > 0)
                return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::lock_guard<std::mutex> lock(mutex_);
    std::cerr << "Seeker not ready after " << timeoutMs << " ms"
              << " (ammo=" << (haveAmmo_ ? "yes" : "no")
              << ", tracks=" << trackCount_ << ")\n";
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
    out.pos      = t.pos;
    out.velocity = t.velocity;
    return out;
}

void SerialTargetSource::start() { started_.store(true); }
void SerialTargetSource::stop()  { running_.store(false); }

void SerialTargetSource::run()
{
    ready_.store(true);

    sensor_link::Parser parser;
    uint8_t buf[512];
    uint8_t type = 0, len = 0, payload[260];

    // Reading begins immediately rather than waiting for start(): the ammo
    // and track count have to arrive before the mission can be released, and
    // they only arrive by reading.
    while (running_.load())
    {
        const ssize_t n = ::read(fd_, buf, sizeof buf);
        if (n < 0)
        {
            if (errno == EINTR) continue;
            std::cerr << "Seeker link read failed: " << std::strerror(errno) << '\n';
            break;
        }

        for (ssize_t i = 0; i < n; ++i)
            if (parser.feed(buf[i], type, payload, len))
                handleFrame(type, payload, len);
    }
}
