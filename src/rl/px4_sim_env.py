from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError as exc:  # pragma: no cover - exercised by users without the optional extra
    raise ImportError("Install the RL extra with `pip install -e .[rl]` to use Px4SimEnv") from exc

from dynamics.quaternion import Quaternion
from dynamics.world import World
from networking.websocket_publisher import GroundTruthWebSocketPublisher
from vehicles.sim_utils import compute_aero_angles_deg


RewardFn = Callable[[dict[str, Any], np.ndarray, "Px4SimEnv"], float]
TerminationFn = Callable[[dict[str, Any], "Px4SimEnv"], bool]
InitialStateSampler = Callable[[np.random.Generator, World], np.ndarray]


def airborne_initial_state(
    rng: np.random.Generator,
    world: World,
    altitude_m: float = 50.0,
    speed_mps: float = 15.0,
    position_xy_std_m: float = 0.0,
    attitude_std_deg: float = 0.0,
) -> np.ndarray:
    _ = world
    y0 = np.zeros(13, dtype=float)
    y0[0:2] = rng.normal(0.0, float(position_xy_std_m), size=2)
    y0[2] = -abs(float(altitude_m))

    attitude_std_rad = np.deg2rad(float(attitude_std_deg))
    euler = rng.normal(0.0, attitude_std_rad, size=3)
    y0[3:7] = Quaternion.euler2quat(euler)
    y0[7] = float(speed_mps)
    return y0


class Px4SimEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        vehicle_model: str = "iris",
        dt: float = 1.0 / 250.0,
        initial_state: str | InitialStateSampler | np.ndarray = "default",
        wind0: np.ndarray | None = None,
        max_episode_time_s: float | None = 20.0,
        terminate_on_ground: bool = False,
        reward_fn: RewardFn | None = None,
        termination_fn: TerminationFn | None = None,
        enable_websocket: bool = False,
        websocket_host: str = "0.0.0.0",
        websocket_port: int = 8765,
        websocket_rate_hz: float = 30.0,
        websocket_system_id: int = 0,
        launch_mode: str = "default",
        model_options: dict[str, Any] | None = None,
    ):
        super().__init__()
        self.dt = float(dt)
        if self.dt <= 0.0:
            raise ValueError("dt must be > 0")

        self.vehicle_model = str(vehicle_model).strip().lower()
        self.initial_state = initial_state
        self.wind0 = np.zeros(6, dtype=float) if wind0 is None else np.asarray(wind0, dtype=float).copy()
        self.max_episode_time_s = max_episode_time_s
        self.terminate_on_ground = bool(terminate_on_ground)
        self.reward_fn = reward_fn
        self.termination_fn = termination_fn
        self.launch_mode = self._normalize_launch_mode(launch_mode)
        if self.launch_mode == "default" and isinstance(initial_state, str):
            if initial_state == "airborne":
                self.launch_mode = "airborne"
            elif initial_state == "catapult":
                self.launch_mode = "catapult"
        self.model_options = {} if model_options is None else dict(model_options)

        self.world = World(vehicle_model=self.vehicle_model, wind0=self.wind0, model_options=self.model_options)
        self.t_us = 0
        self.steps = 0
        self._last_world_out: dict[str, Any] | None = None

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(13,), dtype=np.float32)

        self._ws: GroundTruthWebSocketPublisher | None = None
        self.websocket_system_id = int(websocket_system_id)
        self._websocket_interval_us = max(1, int(1e6 / float(websocket_rate_hz)))
        self._next_websocket_time_us = 0
        if enable_websocket:
            self._ws = GroundTruthWebSocketPublisher(host=websocket_host, port=websocket_port, enabled=True)
            self._ws.start()

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        options = {} if options is None else dict(options)
        self.t_us = 0
        self.steps = 0
        self._next_websocket_time_us = 0

        y0 = options.get("y0")
        if y0 is None:
            y0 = self._make_initial_state(options)
        wind0 = np.asarray(options.get("wind0", self.wind0), dtype=float)
        launch_mode = self._normalize_launch_mode(options.get("launch_mode", self.launch_mode))

        self.world.reset(y0=y0, u0=np.zeros(4), wind0=wind0, t_us=0, launch_mode=launch_mode)

        out = self.world.update(self.t_us, paused=False, freeze_dynamics=True)
        self._last_world_out = out
        self._publish_websocket(out)
        return self._observation(out), self._info(out)

    def step(self, action):
        u = self._action_to_controls(action)
        self.world.set_controls(u)
        self.t_us += int(round(self.dt * 1e6))
        self.steps += 1

        out = self.world.update(self.t_us, paused=False, freeze_dynamics=False)
        self._last_world_out = out
        self._publish_websocket(out)

        obs = self._observation(out)
        reward = self._reward(out, u)
        terminated = self._terminated(out)
        truncated = self._truncated()
        return obs, reward, terminated, truncated, self._info(out)

    def close(self):
        if self._ws is not None:
            self._ws.stop()
            self._ws = None

    def _make_initial_state(self, options: dict[str, Any]) -> np.ndarray | None:
        initial_state = options.get("initial_state", self.initial_state)
        if initial_state is None or (isinstance(initial_state, str) and initial_state in {"default", "catapult"}):
            return None
        if isinstance(initial_state, str) and initial_state == "airborne":
            return airborne_initial_state(
                self.np_random,
                self.world,
                altitude_m=float(options.get("altitude_m", 50.0)),
                speed_mps=float(options.get("speed_mps", 15.0)),
                position_xy_std_m=float(options.get("position_xy_std_m", 0.0)),
                attitude_std_deg=float(options.get("attitude_std_deg", 0.0)),
            )
        if callable(initial_state):
            return np.asarray(initial_state(self.np_random, self.world), dtype=float)
        return np.asarray(initial_state, dtype=float)

    def _normalize_launch_mode(self, launch_mode: str) -> str:
        launch_mode = str(launch_mode).strip().lower()
        if launch_mode not in {"default", "catapult", "airborne", "free"}:
            raise ValueError("launch_mode must be one of default|catapult|airborne|free")
        return launch_mode

    def _action_to_controls(self, action) -> np.ndarray:
        action = np.asarray(action, dtype=float)
        if action.shape != (4,):
            raise ValueError(f"Action must have shape (4,), got {action.shape}")
        action = np.clip(action, -1.0, 1.0)

        if self.vehicle_model == "iris":
            return 0.5 * (action + 1.0)

        u = action.copy()
        u[0] = 0.5 * (action[0] + 1.0)
        return u

    def _observation(self, out: dict[str, Any]) -> np.ndarray:
        return np.asarray(out["y"], dtype=np.float32)

    def _reward(self, out: dict[str, Any], u: np.ndarray) -> float:
        if self.reward_fn is None:
            return 0.0
        return float(self.reward_fn(out, u, self))

    def _terminated(self, out: dict[str, Any]) -> bool:
        y = np.asarray(out["y"], dtype=float)
        if not np.all(np.isfinite(y)):
            return True
        if self.termination_fn is not None and bool(self.termination_fn(out, self)):
            return True
        return self.terminate_on_ground and self.t_us > 0 and float(y[2]) >= 0.0

    def _truncated(self) -> bool:
        if self.max_episode_time_s is None:
            return False
        return (self.t_us / 1e6) >= float(self.max_episode_time_s)

    def _info(self, out: dict[str, Any]) -> dict[str, Any]:
        return {
            "t_us": int(out["t_us"]),
            "y": np.asarray(out["y"], dtype=float).copy(),
            "ydot": np.asarray(out["ydot"], dtype=float).copy(),
            "tau": np.asarray(out["tau"], dtype=float).copy(),
            "sensors": out["sensors"],
        }

    def _publish_websocket(self, out: dict[str, Any]) -> None:
        if self._ws is None or self.t_us < self._next_websocket_time_us:
            return
        y = np.asarray(out["y"], dtype=float)
        z = out["sensors"]
        gps = np.asarray(z["gps"], dtype=float)
        alpha_deg, beta_deg = compute_aero_angles_deg(y, self.world.wind)
        self._ws.publish(
            {
                "system_id": self.websocket_system_id,
                "time_usec": int(self.t_us),
                "u": self.world.u.astype(float).tolist(),
                "position_ned_m": [float(v) for v in y[0:3]],
                "quaternion_wxyz": [float(v) for v in y[3:7]],
                "velocity_body_mps": [float(v) for v in y[7:10]],
                "angular_rate_body_rps": [float(v) for v in y[10:13]],
                "lla": {
                    "lat_deg": float(gps[0]),
                    "lon_deg": float(gps[1]),
                    "alt_m": float(gps[2]),
                },
                "aero": {
                    "alpha_deg": alpha_deg,
                    "beta_deg": beta_deg,
                },
            }
        )
        self._next_websocket_time_us = self.t_us + self._websocket_interval_us
