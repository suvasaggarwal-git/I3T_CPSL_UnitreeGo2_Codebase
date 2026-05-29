#!/usr/bin/env python3
"""
Standalone bridge: subscribes to a Unix datagram socket for (vx, vy, vyaw)
commands and forwards them to the Go2 sport controller via the Unitree SDK2
(CycloneDDS 0.10.2).  Runs in its own process to avoid clashing with the
ROS2 CycloneDDS 0.7.0 that runs in every other node.
"""
import os
import sys
import json
import socket
import time

SDK_PATH = "/home/unitree/jw_go2_rl/cpsl_unitree_sdk2_python"
sys.path.insert(0, SDK_PATH)

from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

SOCKET_PATH = "/tmp/go2_sport.sock"
KEEPALIVE_IDLE_S = 0.5   # send zero-vel if no command received for this long
RECV_TIMEOUT_S   = 0.25  # socket poll interval


def main():
    print("[sport_bridge] Initializing CycloneDDS on eth0...", flush=True)
    ChannelFactoryInitialize(0, "eth0")

    client = SportClient()
    client.SetTimeout(5.0)
    client.Init()
    print("[sport_bridge] SportClient ready", flush=True)

    if os.path.exists(SOCKET_PATH):
        os.unlink(SOCKET_PATH)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    srv.bind(SOCKET_PATH)
    os.chmod(SOCKET_PATH, 0o666)
    srv.settimeout(RECV_TIMEOUT_S)
    print(f"[sport_bridge] Listening on {SOCKET_PATH}", flush=True)

    last_cmd = time.monotonic()

    while True:
        try:
            data = srv.recv(256)
            msg = json.loads(data)
            client.Move(
                float(msg.get("vx",   0.0)),
                float(msg.get("vy",   0.0)),
                float(msg.get("vyaw", 0.0)),
            )
            last_cmd = time.monotonic()
        except socket.timeout:
            if time.monotonic() - last_cmd > KEEPALIVE_IDLE_S:
                client.Move(0.0, 0.0, 0.0)
        except Exception as e:
            print(f"[sport_bridge] Error: {e}", flush=True)


if __name__ == "__main__":
    main()
