import numpy as np

def Rzyx(phi, theta, psi):
    """Return the rotation matrix from ZYX Euler angles."""
    cphi = np.cos(phi)
    sphi = np.sin(phi)
    ctheta = np.cos(theta)
    stheta = np.sin(theta)
    cpsi = np.cos(psi)
    spsi = np.sin(psi)

    R = np.array([
        [cpsi * ctheta, -spsi * cphi + cpsi * stheta * sphi, spsi * sphi + cpsi * cphi * stheta],
        [spsi * ctheta,  cpsi * cphi + spsi * stheta * sphi, -cpsi * sphi + spsi * stheta * cphi],
        [-stheta,        ctheta * sphi,                      ctheta * cphi]
    ])
    
    return R