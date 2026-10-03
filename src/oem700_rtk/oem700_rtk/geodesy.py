"""WGS84 geodetic → local ENU. No ROS imports."""

import math

_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


def geodetic_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    n = _A / math.sqrt(1.0 - _E2 * sl * sl)
    x = (n + alt_m) * cl * co
    y = (n + alt_m) * cl * so
    z = (n * (1.0 - _E2) + alt_m) * sl
    return x, y, z


def geodetic_to_enu(lat_deg, lon_deg, alt_m, lat0, lon0, alt0):
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    x0, y0, z0 = geodetic_to_ecef(lat0, lon0, alt0)
    dx, dy, dz = x - x0, y - y0, z - z0
    lat = math.radians(lat0)
    lon = math.radians(lon0)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    east = -so * dx + co * dy
    north = -sl * co * dx - sl * so * dy + cl * dz
    up = cl * co * dx + cl * so * dy + sl * dz
    return east, north, up


def enu_to_map(east, north, yaw_enu):
    """yaw_enu: radians, map +x relative to East, CCW toward North."""
    c = math.cos(yaw_enu)
    s = math.sin(yaw_enu)
    return c * east + s * north, -s * east + c * north


def base_from_antenna(ant_x, ant_y, yaw, ax, ay):
    """Antenna pose in map minus the lever arm of the antenna in base_footprint."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return ant_x - (c * ax - s * ay), ant_y - (s * ax + c * ay)
