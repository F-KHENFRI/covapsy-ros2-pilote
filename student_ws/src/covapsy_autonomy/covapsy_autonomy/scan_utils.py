"""
Fonctions pures (sans ROS) de traitement d'un scan lidar.

Convention : angles en radians, REP 103 (0 = devant, positif = à gauche).
Attention : le lidar Webots publie ses points avec angle_increment < 0
(de +pi vers -pi). Toutes les fonctions ci-dessous travaillent sur les angles
réels theta_i = angle_min + i * angle_increment, jamais sur les indices.
"""
import math

import numpy as np


def wrap_angle(a):
    """Ramène un angle (ou un tableau d'angles) dans [-pi, pi[."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def scan_to_arrays(ranges, angle_min, angle_increment, range_max):
    """Renvoie (theta, r) en NumPy ; inf → range_max, nan → 0 (mesure ignorée)."""
    r = np.asarray(ranges, dtype=np.float32)
    theta = angle_min + angle_increment * np.arange(r.size, dtype=np.float32)
    r = np.where(np.isinf(r), range_max, r)
    r = np.nan_to_num(r, nan=0.0)
    return wrap_angle(theta).astype(np.float32), r.astype(np.float32)


def sector_mask(theta, a_min, a_max):
    """Masque des angles dans [a_min, a_max] ; gère le secteur arrière (a_min > a_max)."""
    a_min = float(wrap_angle(a_min))
    a_max = float(wrap_angle(a_max))
    if a_min <= a_max:
        return (theta >= a_min) & (theta <= a_max)
    return (theta >= a_min) | (theta <= a_max)       # secteur à cheval sur ±pi


def sector_min(theta, r, a_min, a_max):
    """
    Distance minimale (> 0) dans le secteur angulaire [a_min, a_max] (rad).

    Renvoie +inf si le secteur ne contient aucune mesure valide.
    """
    m = sector_mask(theta, a_min, a_max) & (r > 0.0)
    if not np.any(m):
        return float('inf')
    return float(np.min(r[m]))


def to_points(theta, r):
    """Coordonnées cartésiennes (x, y) dans le repère du lidar ; tableau (N, 2)."""
    return np.stack((r * np.cos(theta), r * np.sin(theta)), axis=-1)


# ---------------------------------------------------------------------------
# Follow the gap (Sezer et Gökasan, 2012)
# ---------------------------------------------------------------------------
