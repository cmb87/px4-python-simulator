#!/usr/bin/env python3
import argparse
import asyncio
import json
import os
import sys
import threading
import time
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

try:
    import websockets
except ImportError as exc:
    raise SystemExit("Please install the 'websockets' package to use this visualizer") from exc


vehicle_dir = os.path.join(os.path.dirname(__file__), "vehicle")
if vehicle_dir not in sys.path:
    sys.path.insert(0, vehicle_dir)

from quaternion import Quaternion


class GroundTruthStore:
    def __init__(self, maxlen: int = 3000):
        self._lock = threading.Lock()
        self.t_s = deque(maxlen=maxlen)
        self.pos_ned = deque(maxlen=maxlen)
        self.quat_wxyz = deque(maxlen=maxlen)

    def append(self, t_usec: int, pos_ned, quat_wxyz) -> None:
        with self._lock:
            self.t_s.append(float(t_usec) * 1e-6)
            self.pos_ned.append(np.asarray(pos_ned, dtype=float))
            q = np.asarray(quat_wxyz, dtype=float)
            qn = np.linalg.norm(q)
            if qn > 0.0:
                q = q / qn
            self.quat_wxyz.append(q)

    def snapshot(self):
        with self._lock:
            if not self.pos_ned:
                return None
            t = np.asarray(self.t_s, dtype=float)
            p = np.asarray(self.pos_ned, dtype=float)
            q = np.asarray(self.quat_wxyz, dtype=float)
            return t, p, q


async def _receiver_loop(uri: str, store: GroundTruthStore) -> None:
    while True:
        try:
            async with websockets.connect(uri, ping_interval=20, ping_timeout=20) as ws:
                print(f"Connected to {uri}")
                async for message in ws:
                    try:
                        payload = json.loads(message)
                        store.append(
                            int(payload["time_usec"]),
                            payload["position_ned_m"],
                            payload["quaternion_wxyz"],
                        )
                    except (KeyError, ValueError, TypeError):
                        continue
        except Exception as err:
            print(f"WebSocket disconnected ({err}), retrying in 1s")
            await asyncio.sleep(1.0)


def start_receiver_thread(uri: str, store: GroundTruthStore) -> threading.Thread:
    def run() -> None:
        asyncio.run(_receiver_loop(uri, store))

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def animate(store: GroundTruthStore) -> None:
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")

    path_line = ax.plot([], [], [], "k-", lw=1.5)[0]
    body_x = ax.plot([], [], [], "r-", lw=2)[0]
    body_y = ax.plot([], [], [], "g-", lw=2)[0]
    body_z = ax.plot([], [], [], "b-", lw=2)[0]

    ax.set_xlabel("North [m]")
    ax.set_ylabel("East [m]")
    ax.set_zlabel("Down [m]")
    ax.set_title("Aircraft Ground Truth Motion (NED)")
    ax.view_init(elev=20, azim=-120)
    ax.invert_zaxis()

    axis_len_m = 2.5
    follow_half_range_m = 30.0

    def update(_frame):
        snap = store.snapshot()
        if snap is None:
            return [path_line, body_x, body_y, body_z]

        _t_s, pos_ned, quat = snap
        north = pos_ned[:, 0]
        east = pos_ned[:, 1]
        down = pos_ned[:, 2]

        path_line.set_data(north, east)
        path_line.set_3d_properties(down)

        p = pos_ned[-1]
        q = quat[-1]
        mgf = Quaternion.Mfg(q).T
        body_axes_ned = mgf @ (np.eye(3) * axis_len_m)

        origin = np.array([p[0], p[1], p[2]])

        ex = np.array([p[0], p[1], p[2]]) + body_axes_ned[:, 0]
        ey = np.array([p[0], p[1], p[2]]) + body_axes_ned[:, 1]
        ez = np.array([p[0], p[1], p[2]]) + body_axes_ned[:, 2]

        body_x.set_data([origin[0], ex[0]], [origin[1], ex[1]])
        body_x.set_3d_properties([origin[2], ex[2]])

        body_y.set_data([origin[0], ey[0]], [origin[1], ey[1]])
        body_y.set_3d_properties([origin[2], ey[2]])

        body_z.set_data([origin[0], ez[0]], [origin[1], ez[1]])
        body_z.set_3d_properties([origin[2], ez[2]])

        ax.set_xlim(origin[0] - follow_half_range_m, origin[0] + follow_half_range_m)
        ax.set_ylim(origin[1] - follow_half_range_m, origin[1] + follow_half_range_m)
        ax.set_zlim(origin[2] - follow_half_range_m, origin[2] + follow_half_range_m)
        return [path_line, body_x, body_y, body_z]

    ani = FuncAnimation(fig, update, interval=40, blit=False)
    _ = ani
    plt.tight_layout()
    plt.show()


def parse_args():
    parser = argparse.ArgumentParser(description="Visualize aircraft ground truth from simulator websocket")
    parser.add_argument("--host", default="127.0.0.1", help="Ground-truth websocket host")
    parser.add_argument("--port", type=int, default=8765, help="Ground-truth websocket port")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    uri = f"ws://{args.host}:{args.port}"
    store = GroundTruthStore(maxlen=4000)
    _thread = start_receiver_thread(uri, store)

    time.sleep(0.2)
    animate(store)


if __name__ == "__main__":
    main()
