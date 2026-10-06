# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Passive Linux Livox capture. Joins existing multicast; never commands the sensor."""

import argparse
from collections import Counter
from contextlib import ExitStack
import json
import select
import socket
import struct
import sys
import time


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("interface")
    parser.add_argument("output")
    parser.add_argument("--seconds", type=float, default=12)
    parser.add_argument("--group", required=True)
    parser.add_argument("--host-ip", required=True)
    parser.add_argument("--sensor-ip", required=True)
    args = parser.parse_args()
    if sys.platform != "linux" or not 0 < args.seconds <= 300:
        parser.error("Requires Linux and a duration in (0, 300] seconds")
    counts: Counter[int] = Counter()
    with ExitStack() as stack:
        listeners = []
        for port in (56301, 56401):
            listener = stack.enter_context(socket.socket(socket.AF_INET, socket.SOCK_DGRAM))
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("", port))
            listener.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                socket.inet_aton(args.group) + socket.inet_aton(args.host_ip),
            )
            listeners.append(listener)
        raw = stack.enter_context(
            socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800))
        )
        raw.bind((args.interface, 0))
        output = stack.enter_context(open(args.output, "xb"))
        output.write(struct.pack("<IHHIIII", 0xA1B23C4D, 2, 4, 0, 0, 65535, 1))
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            ready, _, _ = select.select([raw, *listeners], [], [], 0.25)
            for listener in listeners:
                if listener in ready:
                    listener.recvfrom(65535)
            if raw not in ready:
                continue
            frame = raw.recv(65535)
            stamp = time.time_ns()
            # Untagged Ethernet/IPv4 UDP only; never record other robot traffic.
            if len(frame) < 42 or frame[12:14] != b"\x08\x00" or frame[23] != 17:
                continue
            if socket.inet_ntoa(frame[26:30]) != args.sensor_ip:
                continue
            header = 14 + (frame[14] & 15) * 4
            if len(frame) < header + 8 or struct.unpack_from("!H", frame, 20)[0] & 0x3FFF:
                continue
            source_port = struct.unpack_from("!H", frame, header)[0]
            if source_port not in (56300, 56400):
                continue
            output.write(
                struct.pack("<IIII", stamp // 10**9, stamp % 10**9, len(frame), len(frame))
            )
            output.write(frame)
            counts[source_port] += 1
    print(json.dumps({"saved_packets_by_source_port": counts}))
    if not counts:
        raise RuntimeError("No Livox packets received; verify the existing destination/interface")


if __name__ == "__main__":
    main()
