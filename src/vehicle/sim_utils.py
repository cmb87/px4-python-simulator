import math
import os
from typing import Sequence

import numpy as np


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def controls_to_u(
    latest_controls: tuple[float, ...] | None,
    armed: bool = True,
    size: int = 4,
    clamp_throttle: bool = True,
) -> np.ndarray:
    u = np.zeros(max(int(size), 0), dtype=float)
    if (not armed) or latest_controls is None:
        return u

    for idx in range(min(len(u), len(latest_controls))):
        u[idx] = float(latest_controls[idx])

    if clamp_throttle and len(u) > 3:
        u[3] = clamp(float(latest_controls[3]), 0.0, 1.0)

    return u


def quat_wxyz_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = [float(v) for v in q]
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n <= 0.0:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rot_to_quat_wxyz(rotation: np.ndarray) -> np.ndarray:
    tr = float(np.trace(rotation))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    qn = np.linalg.norm(q)
    return q / qn if qn > 0.0 else np.array([1.0, 0.0, 0.0, 0.0], dtype=float)


def ned_to_enu_position(p_ned: np.ndarray) -> np.ndarray:
    return np.array([p_ned[1], p_ned[0], -p_ned[2]], dtype=float)


def frd_to_flu_vector(v_frd: np.ndarray) -> np.ndarray:
    return np.array([v_frd[0], -v_frd[1], -v_frd[2]], dtype=float)


def ned_frd_quat_to_enu_flu_quat(q_wxyz: np.ndarray) -> np.ndarray:
    r_ned_frd = quat_wxyz_to_rot(q_wxyz)
    t_enu2ned = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]], dtype=float)
    t_frd2flu = np.diag([1.0, -1.0, -1.0])
    r_enu_flu = t_frd2flu @ r_ned_frd @ t_enu2ned
    return rot_to_quat_wxyz(r_enu_flu)


def get_sim_millis(sim_time_us: int) -> int:
    return sim_time_us // 1000


def parse_env_float(name: str, default: float) -> float:
    raw = os.getenv(name, str(default)).strip()
    try:
        return float(raw)
    except ValueError as exc:
        raise ValueError(f"{name} must be a float, got '{raw}'") from exc


def parse_vec3(raw: str, name: str) -> np.ndarray:
    tokens = [tok.strip() for tok in str(raw).split(",") if tok.strip()]
    if len(tokens) != 3:
        raise ValueError(f"{name} must contain 3 comma-separated floats, got '{raw}'")
    try:
        return np.array([float(tokens[0]), float(tokens[1]), float(tokens[2])], dtype=float)
    except ValueError as exc:
        raise ValueError(f"{name} must contain floats, got '{raw}'") from exc


def parse_sim_role(raw: str) -> str:
    role = str(raw).strip().lower()
    if role not in {"standalone", "master", "slave"}:
        raise ValueError(f"SIM_ROLE must be one of standalone|master|slave, got '{raw}'")
    return role


def parse_cutover_mode(raw: str) -> str:
    mode = str(raw).strip().lower()
    if mode not in {"never", "time", "mavlink_cmd"}:
        raise ValueError(f"SIM_TRANSFER_CUTOVER_MODE must be one of never|time|mavlink_cmd, got '{raw}'")
    return mode


def parse_vehicle_model(raw: str, choices: Sequence[str]) -> str:
    model = str(raw).strip().lower()
    normalized_choices = tuple(str(choice).strip().lower() for choice in choices)
    if model not in normalized_choices:
        joined_choices = "|".join(normalized_choices)
        raise ValueError(f"SIM_VEHICLE_MODEL must be one of {joined_choices}, got '{raw}'")
    return model


def parse_arm_frame(raw: str) -> str:
    value = str(raw).strip().lower()
    if value != "master_body":
        raise ValueError(f"SIM_TRANSFER_ARM_FRAME only supports master_body, got '{raw}'")
    return value


def parse_gt_ws_enabled(role: str, raw: str) -> bool:
    value = str(raw).strip().lower()
    if value == "auto":
        return role != "slave"
    if value in {"1", "true", "yes", "on"}:
        return True
    if value in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"SIM_GT_WS_ENABLED must be auto|true|false, got '{raw}'")


def compute_aero_angles_deg(y: np.ndarray, wind: np.ndarray) -> tuple[float | None, float | None]:
    vel_rel = np.asarray(y[7:10], dtype=float) - np.asarray(wind[:3], dtype=float)
    u_r, v_r, w_r = vel_rel
    va = float(np.linalg.norm(vel_rel))
    if va <= 1e-5:
        return None, None

    alpha_deg = float(np.rad2deg(np.arctan2(w_r, u_r)))
    beta_deg = float(np.rad2deg(np.arcsin(np.clip(v_r / va, -1.0, 1.0))))
    return alpha_deg, beta_deg


def compute_airspeed_mps(
    y: np.ndarray,
    wind: np.ndarray,
    pitot_axis_body: np.ndarray | None = None,
) -> tuple[float, float]:
    vel_body = np.asarray(y[7:10], dtype=float)
    wind_body = np.asarray(wind[:3], dtype=float)
    vel_air_body = vel_body - wind_body

    axis = np.array([1.0, 0.0, 0.0], dtype=float) if pitot_axis_body is None else np.asarray(pitot_axis_body, dtype=float)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm <= 1e-9:
        axis = np.array([1.0, 0.0, 0.0], dtype=float)
    else:
        axis = axis / axis_norm

    indicated_airspeed = max(float(np.dot(vel_air_body, axis)), 0.0)
    true_airspeed = float(np.linalg.norm(vel_air_body))
    return indicated_airspeed, true_airspeed
