import logging
import numpy as np

from model.x8 import X8Parameters


logger = logging.getLogger(__name__)


def Smtrx(v):
    """Return the skew-symmetric matrix of a 3-element vector."""
    return np.array(
        [
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0],
        ]
    )


class Parameters(X8Parameters):
    pass


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    P = Parameters()
    logger.info("mass=%s", P.mass)
    logger.info("C_L_alpha=%s", P.C_L_alpha)
