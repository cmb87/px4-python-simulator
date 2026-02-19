import logging
import numpy as np
from Rzyx import Rzyx
from parameters import Parameters
from quaternion import Quaternion
from base_component import SimComponentBase


logger = logging.getLogger(__name__)


def railForces(t, y, u, wind, P):

    pos = y[0:3]            # ned
    quaternions = y[3:7]
    vel = y[7:10]           # in Body frame
    Omega = y[10:13]        # Rates

    
    rel_pos = (pos - P.rail_start_ned)
    rail_dist = np.dot(rel_pos, P.rail_dir_ned)


    rail_pull_max =1.0 # kg


    Force = np.zeros(3)
    Force[0] = rail_pull_max *P.gravity * (P.rail_length - rail_dist)

    Torque = np.zeros(3)

    return np.concatenate([Force, Torque])


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
    y = np.zeros(12)  # Example state vector
    u = np.zeros(4)  # Example control inputs
    wind = np.zeros(6)  # Example wind vector

    
    out = railForces(t, y, u, wind, P)

    logger.info("%s", out)
