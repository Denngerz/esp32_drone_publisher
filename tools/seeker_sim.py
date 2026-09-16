#!/usr/bin/env python3
"""A host stand-in for the ESP32 sensor module.

Speaks the same wire format as firmware/, on a pseudo-terminal instead of a
real UART, so the flight computer's whole serial path can be exercised on one
machine with no board and no wiring. Useful both for developing the Pi side
and for reproducing a run without the hardware present.

It prints the device path to stdout, then streams until interrupted:

    $ python3 tools/seeker_sim.py
    /dev/pts/4
    $ ./build/droneBallistics_Thread_mt --seeker /dev/pts/4

The framing here is written independently of the C++ in
include/link/SensorLink.hpp. That is deliberate: if the two disagree about
the CRC or the layout, nothing decodes, which is a far louder failure than a
shared implementation agreeing with itself.
"""

import argparse
import json
import os
import pty
import struct
import sys
import termios
import time
import tty

MAGIC0 = 0xA5
MAGIC1 = 0x5A

PKT_AMMO = 0x01
PKT_TARGET = 0x02
PKT_STATUS = 0x03


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(pkt_type: int, payload: bytes) -> bytes:
    body = bytes([pkt_type, len(payload)]) + payload
    return bytes([MAGIC0, MAGIC1]) + body + struct.pack("<H", crc16(body))


def ammo_frame(store: dict, hit_radius: float) -> bytes:
    name = store["name"].encode("ascii")[:16].ljust(16, b"\0")
    payload = struct.pack(
        "<16sffff", name,
        float(store["mass"]), float(store["drag"]), float(store["lift"]), hit_radius,
    )
    return encode(PKT_AMMO, payload)


def status_frame(track_count: int) -> bytes:
    return encode(PKT_STATUS, struct.pack("<B", track_count))


def target_frame(t_ms: int, track_id: int, x: float, y: float) -> bytes:
    return encode(PKT_TARGET, struct.pack("<IBff", t_ms, track_id, x, y))


def sample(track, mission_time_s, node_interval_s):
    """Interpolate a track, wrapping at the end, as the firmware does."""
    n = len(track)
    nodes = mission_time_s / node_interval_s
    idx = int(nodes) % n
    nxt = (idx + 1) % n
    frac = nodes - int(nodes)
    a, b = track[idx], track[nxt]
    return (a["x"] + (b["x"] - a["x"]) * frac,
            a["y"] + (b["y"] - a["y"]) * frac)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--targets", default="data/targets.json")
    ap.add_argument("--ammo", default="data/ammo.json")
    ap.add_argument("--store", default="VOG-17", help="which store the bay reports")
    ap.add_argument("--hit-radius", type=float, default=3.0)
    ap.add_argument("--rate", type=float, default=20.0, help="detections per second")
    ap.add_argument("--node-interval", type=float, default=10.0,
                    help="mission seconds between trajectory nodes")
    ap.add_argument("--time-scale", type=float, default=10.0)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to stream, 0 means until interrupted")
    args = ap.parse_args()

    with open(args.targets, encoding="utf-8") as f:
        targets = json.load(f)
    with open(args.ammo, encoding="utf-8") as f:
        catalogue = json.load(f)

    store = next((s for s in catalogue if s["name"] == args.store), None)
    if store is None:
        sys.exit(f"store {args.store!r} is not in {args.ammo}")

    tracks = [t["positions"] for t in targets["targets"]]

    master, slave = pty.openpty()
    # Raw mode: the line discipline would otherwise rewrite bytes that happen
    # to look like newlines or control characters, corrupting binary frames.
    tty.setraw(slave, termios.TCSANOW)

    print(os.ttyname(slave), flush=True)

    period = 1.0 / args.rate
    started = time.monotonic()
    last_slow = 0.0

    try:
        while True:
            now = time.monotonic()
            elapsed = now - started
            if args.duration and elapsed >= args.duration:
                break

            t_ms = int(elapsed * 1000)
            mission_time = elapsed * args.time_scale

            # The bay and the track count repeat so a receiver that starts
            # late still learns them, matching the firmware.
            if elapsed - last_slow >= 1.0 or last_slow == 0.0:
                last_slow = elapsed
                os.write(master, ammo_frame(store, args.hit_radius))
                os.write(master, status_frame(len(tracks)))

            for i, track in enumerate(tracks):
                x, y = sample(track, mission_time, args.node_interval)
                os.write(master, target_frame(t_ms, i, x, y))

            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main()
