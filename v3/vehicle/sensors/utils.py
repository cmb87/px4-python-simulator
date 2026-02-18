import numpy as np
from random import random


# ==================================================
def crandom():
    return random()-0.5


def ned2lla(ned, ref_pos):
    """
    Converts NED (North-East-Down) coordinates to LLA (Latitude, Longitude, Altitude)
    given a reference LLA position.

    Parameters:
    ned (numpy.ndarray): 3x1 array of NED coordinates [N; E; D] in meters
    ref_pos (numpy.ndarray): 3x1 array of reference LLA [latitude, longitude, altitude]
                             (latitude and longitude in degrees, altitude in meters)

    Returns:
    numpy.ndarray: 3x1 array of LLA [latitude, longitude, altitude] in degrees and meters
    """
    # WGS84 ellipsoid constants
    a = 6378137.0  # semi-major axis in meters
    e = 0.081819190842622  # first eccentricity

    # Convert reference position to radians and extract altitude
    ref_lat = np.deg2rad(ref_pos[0])
    ref_lon = np.deg2rad(ref_pos[1])
    ref_alt = ref_pos[2]

    # Radius of curvature in the prime vertical
    N = a / np.sqrt(1 - e**2 * np.sin(ref_lat)**2)

    # Reference ECEF coordinates
    ref_ecef = np.zeros(3)
    ref_ecef[0] = (N + ref_alt) * np.cos(ref_lat) * np.cos(ref_lon)
    ref_ecef[1] = (N + ref_alt) * np.cos(ref_lat) * np.sin(ref_lon)
    ref_ecef[2] = (N * (1 - e**2) + ref_alt) * np.sin(ref_lat)

    # Rotation matrix from NED to ECEF
    Rned2ecef = np.array([
        [-np.sin(ref_lat) * np.cos(ref_lon), -np.sin(ref_lon), -np.cos(ref_lat) * np.cos(ref_lon)],
        [-np.sin(ref_lat) * np.sin(ref_lon),  np.cos(ref_lon), -np.cos(ref_lat) * np.sin(ref_lon)],
        [ np.cos(ref_lat),                   0,               -np.sin(ref_lat)]
    ])

    # Convert NED to ECEF
    ecef_offset = Rned2ecef @ ned
    new_ecef = ref_ecef + ecef_offset

    # Extract ECEF coordinates
    x, y, z = new_ecef
    lon = np.arctan2(y, x)

    # Iterative latitude and altitude calculation
    p = np.sqrt(x**2 + y**2)
    lat = np.arctan2(z, p * (1 - e**2))
    for _ in range(5):
        N = a / np.sqrt(1 - e**2 * np.sin(lat)**2)
        alt = p / np.cos(lat) - N
        lat = np.arctan2(z + e**2 * N * np.sin(lat), p)

    # Convert to degrees
    lat_deg = np.rad2deg(lat)
    lon_deg = np.rad2deg(lon)

    # Normalize longitude to [-180, 180]
    if lon_deg > 180:
        lon_deg -= 360
    elif lon_deg < -180:
        lon_deg += 360

    return np.array([lat_deg, lon_deg, alt])

