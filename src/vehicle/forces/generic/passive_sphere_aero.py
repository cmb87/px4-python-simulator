import numpy as np

from base_component import SimComponentBase


class PassiveSphereAeroForceModel(SimComponentBase):
    def update(self, t_us, paused):
        if paused:
            return self.last_output

        y = self._inputs.get("y")
        wind = self._inputs.get("wind")
        P = self._inputs.get("P")

        if y is None or wind is None or P is None:
            raise ValueError("PassiveSphereAeroForceModel requires inputs: y, wind, P")

        vel_body = np.asarray(y[7:10], dtype=float)
        wind_body = np.asarray(wind[:3], dtype=float)
        v_rel = vel_body - wind_body
        speed = np.linalg.norm(v_rel)

        q = 0.5 * float(P.rho) * speed
        drag_force = -q * float(P.sphere_cd) * float(P.sphere_area) * v_rel

        self.last_output = np.concatenate([drag_force, np.zeros(3)])
        self._last_t_us = int(t_us)
        return self.last_output
