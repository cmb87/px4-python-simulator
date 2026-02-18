from __future__ import annotations


class SimComponentBase:
    def __init__(self):
        self._inputs = {}
        self.last_output = None
        self._last_t_us = None

    def set_inputs(self, **kwargs):
        self._inputs.update(kwargs)

    def _compute_dt_s(self, t_us: int) -> float:
        if self._last_t_us is None:
            self._last_t_us = int(t_us)
            return 0.0
        dt_us = int(t_us) - int(self._last_t_us)
        if dt_us < 0:
            dt_us = 0
        self._last_t_us = int(t_us)
        return dt_us / 1e6

    def update(self, t_us: int, paused: bool):
        raise NotImplementedError
