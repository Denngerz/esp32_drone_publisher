#pragma once
// sensor_uart.hpp — the transmit side of the sensor link.
//
// This board only publishes, so the wrapper is deliberately send-only: it
// frames a payload with sensor_link::encode and writes it. RX is configured
// on the pin anyway, but nothing reads it; adding a command channel later
// means adding a parser here, not re-cabling the board.

#include <cstddef>
#include <cstdint>

namespace sensor_uart
{

class Uart
{
public:
    // Installs the UART driver on the pins from board_config.hpp.
    bool begin();

    // Frames and writes one packet. Silently does nothing if begin() failed,
    // so a missing link degrades into a quiet board rather than a crash.
    void send(uint8_t type, const void* payload, uint8_t payloadLen);

private:
    bool ready_ = false;
};

} // namespace sensor_uart
