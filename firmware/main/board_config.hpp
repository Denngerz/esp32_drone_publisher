#pragma once
// board_config.hpp — the sensor modules' hardware and mission settings.
//
// This board carries two independent modules: the seeker and the payload bay.
// On an airframe they are separate boxes on separate wires, so they are
// separate UARTs here, each with its own task, rather than two roles sharing
// one port. Nothing is shared between them but the chip they happen to run on.
//
// Wiring, ESP32 side -> Raspberry Pi 5 side:
//
//   seeker  UART2 TX  GPIO17  ->  Pi GPIO15  RXD0  (header pin 10)
//   seeker  UART2 RX  GPIO16  <-  Pi GPIO14  TXD0  (header pin  8)
//   bay     UART1 TX  GPIO18  ->  Pi GPIO5   RXD2  (header pin 29)
//   bay     UART1 RX  GPIO19  <-  Pi GPIO4   TXD2  (header pin  7)
//   GND                       <-> GND                (header pin 6, required
//                                  for a common reference; without it the
//                                  links are flaky)
//
// One ground is enough for both links; they share the chip's reference.
//
// The links are one-way in practice: this board only transmits. RX is wired
// and configured anyway so the same harness supports a future command channel
// without re-cabling.
//
// UART1 and UART2 rather than UART0: UART0 goes to the CP2102 bridge that
// appears as /dev/ttyUSB0, and ESP-IDF logs to it. Sharing that port would
// interleave console text with binary frames and corrupt them.
//
// UART1's reset-default pins are GPIO9/GPIO10, which are wired to the SPI
// flash on WROOM modules — driving them does not merely conflict, it stops
// the part booting. The pins below are routed through the GPIO matrix
// instead, which is what uart_set_pin does, so any free pin works. GPIO6-11
// are flash, GPIO34-39 are input-only and cannot be a TX, and GPIO0/2/12/15
// are sampled at reset; the rest are fair game.
//
// On the Pi 5 side the second port needs `dtoverlay=uart2-pi5` in
// /boot/firmware/config.txt; it then appears as /dev/ttyAMA2, while the
// GPIO14/15 UART is /dev/ttyAMA0.

#include <driver/gpio.h>
#include <driver/uart.h>

namespace board
{

// --- links ---
// Both run at the same speed and buffer size; only the port and pins differ.
constexpr int kLinkBaud  = 115200;
constexpr int kLinkBufSz = 2048;

constexpr uart_port_t kSeekerUart  = UART_NUM_2;
constexpr gpio_num_t  kSeekerTxPin = GPIO_NUM_17;
constexpr gpio_num_t  kSeekerRxPin = GPIO_NUM_16;

constexpr uart_port_t kBayUart  = UART_NUM_1;
constexpr gpio_num_t  kBayTxPin = GPIO_NUM_18;
constexpr gpio_num_t  kBayRxPin = GPIO_NUM_19;

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

// --- seeker imperfection ---
// A seeker that reports every target perfectly on every cycle is not a
// seeker, and a flight computer written against one is not tested. These two
// settings are what make the receiver's smoothing and staleness handling
// earn their place. Set both to zero for a noise-free reference run.

// Standard deviation of the position error on each axis, metres. Small
// against the store's lethal radius, large against how far a target moves
// between two reports, which is the regime that makes differencing a
// position into a velocity hard.
constexpr float kDetectionNoiseM = 0.25f;

// Probability that a track drops out on a given reporting round, and how
// long it then stays unreported. The hold is deliberately longer than the
// receiver's staleness threshold, so a dropout is visible as a lost track
// rather than smoothed over as jitter.
constexpr float kTrackDropChance  = 0.01f;
constexpr int   kTrackDropHoldMs  = 700;

} // namespace board
