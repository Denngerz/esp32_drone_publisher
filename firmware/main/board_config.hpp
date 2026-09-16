#pragma once
// board_config.hpp — the sensor module's hardware and mission settings.
//
// Wiring, ESP32 side -> Raspberry Pi side:
//
//   UART2 TX  GPIO17  ->  Pi RXD  (GPIO15, header pin 10)
//   UART2 RX  GPIO16  <-  Pi TXD  (GPIO14, header pin  8)
//   GND               <-> GND     (header pin 6, required for a common
//                                  reference; without it the link is flaky)
//
// The link is one-way in practice: this board only transmits. RX is wired and
// configured anyway so the same harness supports a future command channel
// without re-cabling.
//
// UART2 rather than UART0: UART0 goes to the CP2102 bridge that appears as
// /dev/ttyUSB0, and ESP-IDF logs to it. Sharing that port would interleave
// console text with binary frames and corrupt them.

#include <driver/gpio.h>
#include <driver/uart.h>

namespace board
{

// --- sensor link ---
constexpr uart_port_t kLinkUart  = UART_NUM_2;
constexpr gpio_num_t  kLinkTxPin = GPIO_NUM_17;
constexpr gpio_num_t  kLinkRxPin = GPIO_NUM_16;
constexpr int         kLinkBaud  = 115200;
constexpr int         kLinkBufSz = 2048;

// --- payload bay ---
// What is physically loaded. The bay reports this and its ballistic
// properties; it is a property of the aircraft, not something the flight
// computer tells us, which is why it lives here and not in config.json.
constexpr const char* kLoadedStore = "VOG-17";

// Lethal radius of the loaded store, metres. A property of the munition, so
// the bay is the right place to report it from.
constexpr float kStoreHitRadiusM = 3.0f;

// --- seeker ---
// Mission seconds between two nodes of a target's track. Matches
// targetArrayTimeStep in data/config.json, which is what the tracks were
// sampled at.
constexpr float kNodeIntervalS = 10.0f;

// Wall-clock acceleration. 10 means the mission plays ten times faster than
// real time, matching the simulator's timeScale.
constexpr float kTimeScale = 10.0f;

// How often every held track is reported, in hertz of wall-clock time.
constexpr int kDetectionHz = 20;

// Cadence for the bay report and the seeker status. Both are repeated rather
// than sent once, so a flight computer started late still learns them.
constexpr int kAmmoRepeatMs   = 1000;
constexpr int kStatusRepeatMs = 1000;

} // namespace board
