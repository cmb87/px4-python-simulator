#!/usr/bin/env python3
import argparse
import logging
import math
import time

from visualizer.websockerPublisher import GroundTruthWebSocketPublisher


logger = logging.getLogger(__name__)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dummy ground-truth websocket publisher")
    parser.add_argument("--host", default="0.0.0.0", help="Websocket bind host")
    parser.add_argument("--port", type=int, default=8766, help="Websocket bind port")
    parser.add_argument("--system-id", type=int, default=2, help="system_id in payload")
    parser.add_argument("--rate-hz", type=float, default=20.0, help="Publish rate")
    parser.add_argument("--radius-m", type=float, default=20.0, help="Circular trajectory radius")
    parser.add_argument("--period-s", type=float, default=60.0, help="Seconds per full circle")
    parser.add_argument("--down-m", type=float, default=-5.0, help="Constant NED down position")
    return parser.parse_args()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    args = parse_args()
    period_s = 1.0 / max(args.rate_hz, 1e-3)

    ws = GroundTruthWebSocketPublisher(host=args.host, port=args.port)
    ws.start()

    logger.info(
        "Dummy ground-truth publisher started on ws://%s:%s (system_id=%s, rate=%.2f Hz, radius=%.1f m, period=%.1f s)",
        args.host,
        args.port,
        args.system_id,
        args.rate_hz,
        args.radius_m,
        args.period_s,
    )

    t0 = time.monotonic()

    try:
        while True:
            t = time.monotonic() - t0

            radius_m = max(args.radius_m, 0.0)
            omega = (2.0 * math.pi) / max(args.period_s, 1e-3)
            north = radius_m * math.cos(omega * t)
            east = radius_m * math.sin(omega * t)
            down = args.down_m

            speed_mps = radius_m * omega

            yaw = omega * t + math.pi * 0.5
            q_w = math.cos(0.5 * yaw)
            q_z = math.sin(0.5 * yaw)

            payload = {
                "system_id": int(args.system_id),
                "time_usec": int(time.time() * 1_000_000),
                "u": [0.0, 0.0, 0.0, 0.55, 0.0, 0.0, 0.0, 0.0],
                "position_ned_m": [north, east, down],
                "quaternion_wxyz": [q_w, 0.0, 0.0, q_z],
                "velocity_body_mps": [speed_mps, 0.0, 0.0],
                "angular_rate_body_rps": [0.0, 0.0, omega],
                "lla": {
                    "lat_deg": 47.397742 + north / 111_320.0,
                    "lon_deg": 8.545594 + east / (111_320.0 * math.cos(math.radians(47.397742))),
                    "alt_m": 30.0 - down,
                },
                "aero": {
                    "alpha_deg": 0.0,
                    "beta_deg": 0.0,
                },
            }
            ws.publish(payload)
            time.sleep(period_s)
    except KeyboardInterrupt:
        logger.info("Stopping dummy publisher")
    finally:
        ws.stop()


if __name__ == "__main__":
    main()
