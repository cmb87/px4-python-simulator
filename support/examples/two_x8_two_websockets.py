#!/usr/bin/env python3
"""Run two independent X8 Gym environments with separate websocket outputs."""

import time

try:
    import cv2
except ImportError as exc:
    raise SystemExit("Install OpenCV to run this example: python -m pip install opencv-python") from exc

import numpy as np

from dynamics.quaternion import Quaternion
from perception import PinholeCamera
from rl import Px4SimEnv


def make_airborne_state(north_m: float, east_m: float, altitude_m: float, speed_mps: float) -> np.ndarray:
    y0 = np.zeros(13, dtype=float)
    y0[0:3] = np.array([north_m, east_m, -abs(altitude_m)], dtype=float)
    y0[3:7] = Quaternion.euler2quat(np.zeros(3))
    y0[7] = speed_mps
    return y0


def format_camera_measurement(label: str, measurement) -> str:
    pixel = np.array2string(measurement.pixel, precision=1)
    return f"{label}: visible={measurement.visible} pixel={pixel} depth={measurement.depth_m:.1f}m"


def render_camera_frame(label: str, measurement, width: int, height: int) -> np.ndarray:
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    center = (width // 2, height // 2)
    cv2.drawMarker(frame, center, (90, 90, 90), cv2.MARKER_CROSS, 20, 1)
    cv2.putText(frame, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)

    if measurement.visible:
        pixel = tuple(np.rint(measurement.pixel).astype(int))
        cv2.circle(frame, pixel, 10, (0, 0, 255), -1)
        cv2.circle(frame, pixel, 14, (255, 255, 255), 1)
        cv2.putText(
            frame,
            f"target depth={measurement.depth_m:.1f}m",
            (12, height - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            1,
        )
    else:
        cv2.putText(
            frame,
            "target not visible",
            (12, height - 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 165, 255),
            1,
        )

    return frame


def main() -> None:
    env0 = Px4SimEnv(
        vehicle_model="x8",
        initial_state="airborne",
        launch_mode="airborne",
        enable_websocket=True,
        websocket_host="0.0.0.0",
        websocket_port=8765,
        websocket_rate_hz=30.0,
        websocket_system_id=1,
        max_episode_time_s=None,
    )
    env1 = Px4SimEnv(
        vehicle_model="x8",
        initial_state="airborne",
        launch_mode="airborne",
        enable_websocket=True,
        websocket_host="0.0.0.0",
        websocket_port=8766,
        websocket_rate_hz=30.0,
        websocket_system_id=2,
        max_episode_time_s=None,
    )
    camera = PinholeCamera(width=640, height=480, fx=400.0, fy=400.0)

    try:
        _, info0 = env0.reset(options={"y0": make_airborne_state(0.0, 0.0, 80.0, 18.0)})
        _, info1 = env1.reset(options={"y0": make_airborne_state(35.0, 25.0, 90.0, 18.0)})

        print("Vehicle 0 websocket: ws://127.0.0.1:8765 system_id=1")
        print("Vehicle 1 websocket: ws://127.0.0.1:8766 system_id=2")
        print(f"Vehicle 0 initial state: {info0['y']}")
        print(f"Vehicle 1 initial state: {info1['y']}")

        # X8 action convention is normalized: action[0] maps from [-1, 1] to throttle [0, 1].
        action0 = np.array([-1, 0.02, -0.02, 0.0], dtype=np.float32)
        action1 = np.array([-1, 0.02, -0.02, 0.0], dtype=np.float32)
        next_state_print_s = 1.0
        cv2.namedWindow("Camera 0 -> Vehicle 1", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Camera 1 -> Vehicle 0", cv2.WINDOW_NORMAL)

        while True:
            _, reward0, terminated0, truncated0, info0 = env0.step(action0)
            _, reward1, terminated1, truncated1, info1 = env1.step(action1)

            p0 = info0["y"][0:3]
            p1 = info1["y"][0:3]
            cam0_sees_1 = camera.project_world_point(
                point_ned=p1,
                ego_position_ned=p0,
                ego_quaternion_wxyz=info0["y"][3:7],
            )
            cam1_sees_0 = camera.project_world_point(
                point_ned=p0,
                ego_position_ned=p1,
                ego_quaternion_wxyz=info1["y"][3:7],
            )
            cv2.imshow(
                "Camera 0 -> Vehicle 1",
                render_camera_frame("Camera 0 -> Vehicle 1", cam0_sees_1, camera.width, camera.height),
            )
            cv2.imshow(
                "Camera 1 -> Vehicle 0",
                render_camera_frame("Camera 1 -> Vehicle 0", cam1_sees_0, camera.width, camera.height),
            )
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                print("Camera display requested exit")
                break

            t_s = info0["t_us"] / 1e6
            if t_s >= next_state_print_s:
                separation_m = float(np.linalg.norm(p1 - p0))
                print(
                    "t={:.1f}s p0={} p1={} separation={:.1f}m rewards=({:.3f}, {:.3f}) {} {}".format(
                        t_s,
                        np.array2string(p0, precision=2),
                        np.array2string(p1, precision=2),
                        separation_m,
                        reward0,
                        reward1,
                        format_camera_measurement("cam0->veh1", cam0_sees_1),
                        format_camera_measurement("cam1->veh0", cam1_sees_0),
                    )
                )
                next_state_print_s += 1.0

            if terminated0 or truncated0 or terminated1 or truncated1:
                print("At least one episode finished")
                break
            time.sleep(env0.dt)
    finally:
        env0.close()
        env1.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
