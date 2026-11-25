"""
Petites fonctions pour exploiter un sensor_msgs/LaserScan (NumPy, sans ROS).

Conventions REP 103 : angle 0 = devant, angle > 0 = à GAUCHE, en radians.
Le lidar tourne en sens horaire (angle_increment < 0) : scan_vers_tableaux remet
les points dans l'ordre des angles croissants.
"""
import math

import numpy as np


def scan_vers_tableaux(msg):
    """LaserScan -> (angles, distances) triés par angle croissant ; inf -> portée max."""
    d = np.asarray(msg.ranges, dtype=np.float64)
    a = msg.angle_min + msg.angle_increment * np.arange(d.size)
    a = (a + math.pi) % (2 * math.pi) - math.pi          # ramené dans [-pi, pi[
    d = np.where(np.isinf(d), msg.range_max, d)
    ok = np.isfinite(d) & (d >= max(msg.range_min, 1e-3))
    a, d = a[ok], d[ok]
    ordre = np.argsort(a)
    return a[ordre], d[ordre]


def distance_secteur(angles, distances, deg_min, deg_max):
    """Distance minimale (m) dans le secteur [deg_min, deg_max] (degrés) ; inf si vide."""
    a0, a1 = math.radians(deg_min), math.radians(deg_max)
    m = (angles >= a0) & (angles <= a1) if a0 <= a1 else (angles >= a0) | (angles <= a1)
    return float(np.min(distances[m])) if np.any(m) else math.inf


def lisser(valeurs, k):
    """Moyenne glissante sur k points (même taille en sortie)."""
    if k <= 1 or len(valeurs) == 0:
        return np.asarray(valeurs, dtype=float)
    pad = np.pad(np.asarray(valeurs, dtype=float), (k // 2, k - 1 - k // 2), mode='edge')
    return np.convolve(pad, np.ones(k) / k, mode='valid')


def limiter(x, x_min, x_max):
    """Sature x dans [x_min, x_max]."""
    return max(x_min, min(x_max, x))
