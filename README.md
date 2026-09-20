# esp32_drone_publisher

A drone ballistics simulator split across two machines. A Raspberry Pi 5 runs
the flight computer, and an ESP32 acts as the aircraft's sensor hardware: the
seeker that observes targets and the payload bay that reports what is loaded.
Each is a module with its own UART link to the Pi.

The point of the split is what it takes away. In the original single-machine
simulator the mission read `data/targets.json` and knew every target's entire
future, interpolating it at will. Here it knows only what the seeker has told
it so far, with measurement error, with tracks that sometimes drop out, and
with a link that can go silent. That is the situation on an airframe.

## Architecture

**ESP32, `firmware/`.** Two peripherals that only transmit, on one chip. The
seeker publishes a detection for each track it holds and how many tracks that
is; the payload bay publishes the ballistic properties of the loaded store.
They are separate modules on separate UARTs with a task each, because on an
airframe they are separate boxes that fail separately. Neither accepts
commands.

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
`SerialTargetSource` listens to the seeker, and the mission cannot tell which
it has. `SerialAmmoSource` is the bay's counterpart; it is deliberately not an
`ITargetSource`, because the bay answers a question that cannot change in
flight.

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
idf.py -p /dev/ttyUSB0 monitor
```

`/dev/ttyUSB0` is the board's USB console, used for flashing and logs. It is
not the link.

Keep those two commands separate. `-b` is the flashing speed, and passing it to
`monitor` as well makes it read the 115200-baud console at 460800, which fills
the screen with binary noise that looks exactly like a board stuck in a boot
loop.

The track and store data in `data/*.json` is converted into constant arrays by
`tools/gen_mission_data.py` during the firmware build rather than parsed on the
device. Edit the JSON and rebuild; CMake reruns the generator.

## Wiring

Two links, one per module.

| Module | ESP32 | Raspberry Pi 5 | Pi device |
| --- | --- | --- | --- |
| seeker | GPIO17, UART2 TX | GPIO15, header pin 10 | `/dev/ttyAMA0` |
| seeker | GPIO16, UART2 RX | GPIO14, header pin 8 | |
| bay | GPIO18, UART1 TX | GPIO5, header pin 29 | `/dev/ttyAMA2` |
| bay | GPIO19, UART1 RX | GPIO4, header pin 7 | |
| both | GND | header pin 6 | |

One ground serves both links, but it is required, not optional; without it they
are unreliable. The RX lines are wired and configured although nothing reads
them, so a command channel can be added later without re-cabling.

The board transmits on UART1 and UART2 rather than UART0 because UART0 is the
USB console that ESP-IDF logs to, and mixing log text into binary frames
corrupts them.

**Do not use UART1's reset-default pins, GPIO9 and GPIO10.** They are wired to
the SPI flash on WROOM modules, and driving them stops the part booting at all.
The pins above are routed through the GPIO matrix instead, which is what
`uart_set_pin` does, so any free pin works: GPIO6-11 are flash, GPIO34-39 are
input-only and cannot be a TX, GPIO0/2/12/15 are sampled at reset, and the rest
are fair game.

The two TX lines must go to two separate Pi pins. Tying both to one RX would
put two push-pull drivers on one wire, which is a bus conflict rather than a
shortcut.

## Running against the board

The Pi's serial port must be freed from the login console first, and the
second UART enabled:

```bash
sudo raspi-config nonint do_serial_cons 1
sudo raspi-config nonint do_serial_hw 0
echo 'dtoverlay=uart2-pi5' | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

`uart2-pi5` is what puts a UART on GPIO4/GPIO5; it appears as `/dev/ttyAMA2`.
Check both ports came up with `ls -l /dev/ttyAMA*`, and that the pins really
switched function with `pinctrl get 4,5`.

**On a Pi 5, use `/dev/ttyAMA0`, not `/dev/serial0`.** The `serial0` symlink
points at the dedicated debug connector, not at the GPIO header. `ttyAMA0` is
the GPIO14/15 UART and appears once `dtparam=uart0=on` is set. Pointing the
mission at `serial0` produces silence and looks exactly like bad wiring.

```bash
./build/droneBallistics_Thread_mt --seeker /dev/ttyAMA0 --bay /dev/ttyAMA2
```

`--seeker` alone still works and expects one module publishing everything on
one wire, which is what the firmware did before the bay was split out. With
`--bay` the store report is read from its own link instead, and the seeker's
link is not waited on for one.

Add `table` to use the lookup-table solver instead of the analytical one. Omit
both flags to fall back to replaying `data/targets.json` locally.

The process exits non-zero if the link goes silent mid-mission. `simulation.json`
is written either way, so a partial run can still be inspected.

## Running without the board

`tools/seeker_sim.py` is a host stand-in that speaks the same protocol on a
pseudo-terminal, with the same noise and dropout defaults as the firmware:

```bash
python3 tools/seeker_sim.py        # prints a device path, then streams
./build/droneBallistics_Thread_mt --seeker /dev/pts/N
```

`--split` stands in for both modules on two links, as the board wires them,
printing the seeker's path first and the bay's second:

```bash
python3 tools/seeker_sim.py --split
./build/droneBallistics_Thread_mt --seeker /dev/pts/N --bay /dev/pts/M
```

Pass `--noise 0 --drop-chance 0` for a clean reference run. Its framing is
written independently of the C++ header on purpose: if the two ever disagree
about the CRC or the layout, nothing decodes at all, which is a much louder
failure than one implementation agreeing with itself.

## Tests

`ctest` covers the protocol and both receivers. The protocol suite exercises
what a real serial line produces: a listener joining mid-stream, frames split
across reads, and a corrupt frame that must be dropped without wedging the
parser. The seeker's receiver suite runs the real `SerialTargetSource` against
a pseudo-terminal and checks that a velocity is inferred from successive
detections, that a track which stops being reported holds its position but
loses its velocity, and that prolonged silence marks the link unhealthy. The
bay's suite checks the opposite kind of property: that the store is learned
from a link of its own, and that seeker traffic arriving on the bay's wire —
which means the modules are crossed — is ignored rather than misread.

## Layout

```
data/            track and store data, the single source of truth
firmware/        ESP-IDF project for the ESP32
firmware/main/   seeker.cpp and bay.cpp: the two modules, a task each
include/link/    the wire protocol, shared by both halves
include/mt/      threaded components: physics, mission, sensor receivers
src/             flight computer implementation
tests/           host tests
tools/           the data generator and the host stand-in
```
