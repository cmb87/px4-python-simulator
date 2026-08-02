#!/usr/bin/env python3
"""Run the X8 Gym environment with websocket ground-truth output enabled."""

import time

import numpy as np

from rl import Px4SimEnv


def main() -> None:
    altitude_m = 80.0
    speed_mps = 18.0

    env = Px4SimEnv(
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
    try:
        print(f"Starting new episode with altitude={altitude_m:.1f} m and speed={speed_mps:.1f} m/s")
        obs, info = env.reset(options={"altitude_m": altitude_m, "speed_mps": speed_mps})
        print(f"Initial state from obs: {obs}")
        print(f"Initial state from info['y']: {info['y']}")

        # X8 action convention is normalized: action[0] maps from [-1, 1] to throttle [0, 1].
        action = np.array([0.35, 0.0, 0.0, 0.0], dtype=np.float32)
        next_state_print_s = 1.0
        while True:
            obs, reward, terminated, truncated, info = env.step(action)
            if info["t_us"] / 1e6 >= next_state_print_s:
                state = info["y"]
                position_ned = state[0:3]
                velocity_body = state[7:10]
                print(
                    "t={:.1f}s pos_ned={} vel_body={} reward={:.3f}".format(
                        info["t_us"] / 1e6,
                        np.array2string(position_ned, precision=2),
                        np.array2string(velocity_body, precision=2),
                        reward,
                    )
                )
                next_state_print_s += 1.0
            if terminated or truncated:
                print("Episode finished")
                break
            time.sleep(env.dt)
    finally:
        env.close()


if __name__ == "__main__":
    main()
