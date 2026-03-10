import numpy as np


def make_initial_state(config=None):
    _ = config
    y0 = np.zeros(13)
    y0[0:3] = np.array([0.0, 0.0, 0.0])
    y0[3] = 1.0
    return y0
