#pragma once
// sensor_uart.hpp — the transmit side of one sensor link.
//
// A module only publishes, so the wrapper is deliberately send-only: it
// frames a payload with sensor_link::encode and writes it. RX is configured
// on the pin anyway, but nothing reads it; adding a command channel later
// means adding a parser here, not re-cabling the board.
//
// The port and pins are constructor arguments rather than constants, because
// this board brings up two of these: one for the seeker and one for the bay.
// Each instance owns its UART outright, so two tasks writing to two objects
// never share a buffer and need no lock between them.

#include <cstddef>
#include <cstdint>

#include <driver/gpio.h>
#include <driver/uart.h>

namespace sensor_uart
{

class Uart
{
public:
    // label names the module in log lines: "seeker", "bay".
    Uart(uart_port_t port, gpio_num_t txPin, gpio_num_t rxPin, const char* label)
        : port_(port), txPin_(txPin), rxPin_(rxPin), label_(label) {}

    // Installs the UART driver on this instance's port and pins.
    bool begin();

    // Frames and writes one packet. Silently does nothing if begin() failed,
    // so a missing link degrades into a quiet module rather than a crash.
    void send(uint8_t type, const void* payload, uint8_t payloadLen);

private:
    uart_port_t port_;
    gpio_num_t  txPin_;
    gpio_num_t  rxPin_;
    const char* label_;
    bool        ready_ = false;
};

} // namespace sensor_uart
