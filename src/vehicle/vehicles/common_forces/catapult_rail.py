import logging

import numpy as np

from ...base_component import SimComponentBase
from ...parameters import Parameters
from ...quaternion import Quaternion


logger = logging.getLogger(__name__)


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= 1e-9:
        raise ValueError("Rail direction vector must be non-zero")
    return np.asarray(v, dtype=float) / n


def get_rail_distance_m(y, P) -> float:
    pos = np.asarray(y[0:3], dtype=float)
    rail_start_ned = np.asarray(P.rail_start_ned, dtype=float)
    rail_dir_ned = _unit(np.asarray(P.rail_dir_ned, dtype=float))
    rel_pos = pos - rail_start_ned
    return float(np.dot(rel_pos, rail_dir_ned))


def rail_alignment_quaternion_wxyz(P) -> np.ndarray:
    rail_dir = _unit(np.asarray(P.rail_dir_ned, dtype=float))
    yaw = float(np.arctan2(rail_dir[1], rail_dir[0]))
    horiz = float(np.hypot(rail_dir[0], rail_dir[1]))
    pitch = float(np.arctan2(-rail_dir[2], max(horiz, 1e-9)))
    return Quaternion.euler2quat(np.asarray([0.0, pitch, yaw], dtype=float))


def railForces(t, y, u, wind, P):
    _ = t
    _ = y
    _ = u
    _ = wind

    rail_dist = get_rail_distance_m(y, P)
    rail_pull_max = float(getattr(P, "rail_pull_max", 1.0))
    rail_length = float(getattr(P, "rail_length", 0.0))

    rail_force_ned = rail_pull_max * float(P.gravity) * (rail_length - rail_dist)
    if rail_force_ned < 0.0:
        rail_force_ned = 0.0

    rail_dir_ned = _unit(np.asarray(P.rail_dir_ned, dtype=float))
    quat = rail_alignment_quaternion_wxyz(P)
    mfg = Quaternion.Mfg(quat)
    force_body = mfg @ (rail_force_ned * rail_dir_ned)

    return np.concatenate([force_body, np.zeros(3)])


class CatapultRailForceModel(SimComponentBase):
    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        u = self._inputs.get("u")
        wind = self._inputs.get("wind")
        P = self._inputs.get("P")

        if y is None or u is None or wind is None or P is None:
            raise ValueError("CatapultRailForceModel requires inputs: y, u, wind, P")

        t_s = float(t_us) / 1e6
        self.last_output = railForces(t_s, y, u, wind, P)
        self._last_t_us = int(t_us)
        return self.last_output


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    P = Parameters()
    t = 0.0
    y = np.zeros(12)
    u = np.zeros(4)
    wind = np.zeros(6)

    out = railForces(t, y, u, wind, P)

    logger.info("%s", out)
