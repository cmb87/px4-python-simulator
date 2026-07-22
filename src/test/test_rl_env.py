import numpy as np
import pytest

pytest.importorskip("gymnasium")

from rl import Px4SimEnv


def test_rl_env_resets_without_px4_or_websocket():
    env = Px4SimEnv(vehicle_model="iris")
    try:
        obs, info = env.reset()
        assert obs.shape == env.observation_space.shape
        assert info["y"].shape == (13,)
        assert env._ws is None
    finally:
        env.close()


def test_rl_env_steps_and_advances_time():
    env = Px4SimEnv(vehicle_model="iris", max_episode_time_s=1.0)
    try:
        env.reset()
        obs, reward, terminated, truncated, info = env.step(np.zeros(4, dtype=np.float32))
        assert obs.shape == (13,)
        assert reward == 0.0
        assert not terminated
        assert not truncated
        assert info["t_us"] > 0
    finally:
        env.close()


def test_rl_env_airborne_reset_starts_above_ground():
    env = Px4SimEnv(vehicle_model="iris", initial_state="airborne")
    try:
        obs, info = env.reset(options={"altitude_m": 25.0, "speed_mps": 0.0})
        assert obs[2] < 0.0
        assert np.isclose(info["y"][2], -25.0)
    finally:
        env.close()


def test_rl_env_reset_clears_time_between_episodes():
    env = Px4SimEnv(vehicle_model="iris")
    try:
        env.reset()
        env.step(np.zeros(4, dtype=np.float32))
        assert env.t_us > 0

        _, info = env.reset()
        assert env.t_us == 0
        assert info["t_us"] == 0
    finally:
        env.close()


def test_rl_env_accepts_explicit_initial_state_array():
    y0 = np.zeros(13, dtype=float)
    y0[2] = -10.0
    y0[3] = 1.0
    env = Px4SimEnv(vehicle_model="iris", initial_state=y0)
    try:
        obs, _ = env.reset()
        assert np.isclose(obs[2], -10.0)
    finally:
        env.close()


def test_x8_catapult_launch_mode_starts_on_rail():
    env = Px4SimEnv(vehicle_model="x8", initial_state="catapult")
    try:
        obs, _ = env.reset()
        assert env.world.rail_launch_enabled
        assert not env.world.P.left_rail
        assert np.allclose(obs[0:3], env.world.P.rail_start_ned)
    finally:
        env.close()


def test_x8_airborne_launch_mode_preserves_airborne_state():
    env = Px4SimEnv(vehicle_model="x8", initial_state="airborne")
    try:
        obs, _ = env.reset(options={"altitude_m": 75.0, "speed_mps": 18.0})
        assert env.world.rail_launch_enabled
        assert env.world.P.left_rail
        assert np.isclose(obs[2], -75.0)
        assert np.isclose(obs[7], 18.0)
    finally:
        env.close()


def test_x8_free_launch_mode_disables_rail():
    env = Px4SimEnv(vehicle_model="x8", launch_mode="free")
    try:
        env.reset()
        assert not env.world.rail_launch_enabled
        assert env.world.P.left_rail
    finally:
        env.close()
