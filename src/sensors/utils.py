import numpy as np
from random import random


# ==================================================
def crandom():
    return random()-0.5


def ned2lla(ned, ref_pos):
    """
    Converts NED (North-East-Down) coordinates to LLA (Latitude, Longitude, Altitude)
    given a reference LLA position using the same spherical map projector model
    used by jMAVSim (GlobalPositionProjector).

    Parameters:
    ned (numpy.ndarray): 3x1 array of NED coordinates [N; E; D] in meters
    ref_pos (numpy.ndarray): 3x1 array of reference LLA [latitude, longitude, altitude]
                             (latitude and longitude in degrees, altitude in meters)

    Returns:
    numpy.ndarray: 3x1 array of LLA [latitude, longitude, altitude] in degrees and meters
    """
    # jMAVSim-compatible spherical projection (GlobalPositionProjector.reproject)
    r_earth = 6371000.0
    lat0 = np.deg2rad(ref_pos[0])
    lon0 = np.deg2rad(ref_pos[1])
    alt0 = ref_pos[2]

    x_rad = ned[0] / r_earth
    y_rad = ned[1] / r_earth
    c = np.sqrt(x_rad * x_rad + y_rad * y_rad)

    if c != 0.0:
        sin_c = np.sin(c)
        cos_c = np.cos(c)
        sin_lat0 = np.sin(lat0)
        cos_lat0 = np.cos(lat0)
        lat = np.arcsin(cos_c * sin_lat0 + (x_rad * sin_c * cos_lat0) / c)
        lon = lon0 + np.arctan2(y_rad * sin_c, c * cos_lat0 * cos_c - x_rad * sin_lat0 * sin_c)
    else:
        lat = lat0
        lon = lon0

    alt = alt0 - ned[2]

    return np.array([np.rad2deg(lat), np.rad2deg(lon), alt])
