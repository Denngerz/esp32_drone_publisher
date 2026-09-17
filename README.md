# esp32_drone_publisher

A drone ballistics simulator split across two machines. A Raspberry Pi 5 runs
the flight computer, and an ESP32 acts as the aircraft's sensor hardware: the
seeker that observes targets and the payload bay that reports what is loaded.
They talk over a UART link.

The point of the split is what it takes away. In the original single-machine
simulator the mission read `data/targets.json` and knew every target's entire
future, interpolating it at will. Here it knows only what the seeker has told
it so far, with measurement error, with tracks that sometimes drop out, and
with a link that can go silent. That is the situation on an airframe.

## Architecture

**ESP32, `firmware/`.** A peripheral that only transmits. It publishes three
things: detections for each track it holds, the ballistic properties of the
loaded store, and how many tracks it is holding. It accepts no commands.

**Raspberry Pi, `src/`.** Physics, the mission state machine, the ballistic
solver and the target source. `data/config.json` stays a local file, because
mission configuration is not something a peripheral owns. Ammunition does come
from the board: the bay reports what is physically loaded, and that overrides
the ammo name in the config.

**The link, `include/link/SensorLink.hpp`.** Self-synchronising frames with a
CRC, so a listener that joins mid-stream or drops bytes recovers on its own.
Detections carry the seeker's own clock rather than relying on arrival times,
which is what lets the receiver difference two positions into a velocity even
when frames are delayed or arrive bunched together.

`ITargetSource` is the seam. `ThreadSafeTargetProvider` replays a local file,
`SerialTargetSource` listens to the board, and the mission cannot tell which
it has.

## Building

The Pi side needs CMake and a C++20 compiler. Dependencies are fetched during
configure, so nothing has to be installed system-wide:

```bash
cmake -S . -B build
cmake --build build -j4
ctest --test-dir build
```

The firmware needs ESP-IDF v6.1 and builds for the classic ESP32:

```bash
source ~/.espressif/tools/activate_idf_v6.1.sh
cd firmware
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 -b 460800 flash
```

`/dev/ttyUSB0` is the board's USB console, used for flashing and logs. It is
not the link.

The track and store data in `data/*.json` is converted into constant arrays by
`tools/gen_mission_data.py` during the firmware build rather than parsed on the
device. Edit the JSON and rebuild; CMake reruns the generator.

## Wiring

| ESP32 | Raspberry Pi 5 |
| --- | --- |
| GPIO17, UART2 TX | GPIO15, header pin 10 |
| GPIO16, UART2 RX | GPIO14, header pin 8 |
| GND | header pin 6 |

A common ground is required, not optional; without it the link is unreliable.

The board transmits on UART2 rather than UART0 because UART0 is the USB console
that ESP-IDF logs to, and mixing log text into binary frames corrupts them.

## Running against the board

The Pi's serial port must be freed from the login console first:

```bash
sudo raspi-config nonint do_serial_cons 1
sudo raspi-config nonint do_serial_hw 0
sudo reboot
```

**On a Pi 5, use `/dev/ttyAMA0`, not `/dev/serial0`.** The `serial0` symlink
points at the dedicated debug connector, not at the GPIO header. `ttyAMA0` is
the GPIO14/15 UART and appears once `dtparam=uart0=on` is set. Pointing the
mission at `serial0` produces silence and looks exactly like bad wiring.

```bash
./build/droneBallistics_Thread_mt --seeker /dev/ttyAMA0
```

Add `table` to use the lookup-table solver instead of the analytical one. Omit
`--seeker` to fall back to replaying `data/targets.json` locally.

The process exits non-zero if the link goes silent mid-mission. `simulation.json`
is written either way, so a partial run can still be inspected.

## Running without the board

`tools/seeker_sim.py` is a host stand-in that speaks the same protocol on a
pseudo-terminal, with the same noise and dropout defaults as the firmware:

```bash
python3 tools/seeker_sim.py        # prints a device path, then streams
./build/droneBallistics_Thread_mt --seeker /dev/pts/N
```

Pass `--noise 0 --drop-chance 0` for a clean reference run. Its framing is
written independently of the C++ header on purpose: if the two ever disagree
about the CRC or the layout, nothing decodes at all, which is a much louder
failure than one implementation agreeing with itself.

## Tests

`ctest` covers the two halves of the link. The protocol suite exercises what a
real serial line produces: a listener joining mid-stream, frames split across
reads, and a corrupt frame that must be dropped without wedging the parser. The
receiver suite runs the real `SerialTargetSource` against a pseudo-terminal and
checks that a velocity is inferred from successive detections, that a track
which stops being reported holds its position but loses its velocity, and that
prolonged silence marks the link unhealthy.

## Layout

```
data/            track and store data, the single source of truth
firmware/        ESP-IDF project for the ESP32
include/link/    the wire protocol, shared by both halves
include/mt/      threaded components: physics, mission, target sources
src/             flight computer implementation
tests/           host tests
tools/           the data generator and the host stand-in
```
