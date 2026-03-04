import math

import numpy as np


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def controls_to_u(latest_controls: tuple[float, ...] | None, armed: bool) -> np.ndarray:
    u = np.zeros(4)
    if (not armed) or latest_controls is None:
        return u

    for idx in range(min(3, len(latest_controls))):
        u[idx] = float(latest_controls[idx])

    if len(latest_controls) > 3:
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
