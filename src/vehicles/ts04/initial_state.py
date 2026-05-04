import numpy as np


def make_initial_state(config=None):
    cfg = {} if config is None else dict(config)
    pitch90 = bool(cfg.get("ts04_pitch90_start", False))

    y0 = np.zeros(13)
    y0[0:3] = np.array([0.0, 0.0, 0.0])
    if pitch90:
        y0[3:7] = np.array([np.sqrt(0.5), 0.0, np.sqrt(0.5), 0.0])
    else:
        y0[3] = 1.0
    return y0
