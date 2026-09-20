#include "../../include/mt/SerialLink.hpp"
#include "../../include/link/SensorLink.hpp"
#include "../../include/mt/ConsoleLog.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

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

SerialLink::SerialLink(std::string device, int baud, std::string label)
    : device_(std::move(device)), baud_(baud), label_(std::move(label))
{
}

SerialLink::~SerialLink()
{
    if (fd_ >= 0)
        ::close(fd_);
}

bool SerialLink::open()
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

    console::line(label_ + " link open on " + device_ + " @ " +
                  std::to_string(baud_) + " baud");
    return true;
}

void SerialLink::stop() { running_.store(false); }

void SerialLink::readUntilStopped(const FrameSink& onFrame)
{
    sensor_link::Parser parser;
    uint8_t buf[512];
    uint8_t type = 0, len = 0, payload[260];

    while (running_.load())
    {
        const ssize_t n = ::read(fd_, buf, sizeof buf);
        if (n < 0)
        {
            if (errno == EINTR) continue;
            console::error(label_ + " link read failed: " + std::strerror(errno));
            break;
        }

        for (ssize_t i = 0; i < n; ++i)
            if (parser.feed(buf[i], type, payload, len))
                onFrame(type, payload, len);
    }
}
