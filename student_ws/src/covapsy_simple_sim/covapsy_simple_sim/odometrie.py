"""
Capteurs proprioceptifs simulés : centrale inertielle (IMU) et odométrie embarquée.

Ce sont des MESURES de la voiture, pas la vérité terrain :
  * gyroscope : vitesse de lacet réelle + biais (constant par voiture) + bruit ;
  * accéléromètre : accélérations longitudinale (dv/dt) et latérale (v · lacet) + bruit ;
  * odomètre de roue : vitesse réelle × (1 + erreur d'échelle) + bruit ; il ne voit pas
    les glissements (sous-virage, voiture qui frotte un mur) ;
  * odométrie (estimation à l'estime) : intégration de la vitesse de roue et du gyroscope,
    à partir de (0, 0, 0) sur la grille. Elle DÉRIVE avec le temps, comme sur la vraie
    voiture : ce n'est pas une position absolue sur la piste (règlement CoVAPSy).

Module pur (sans ROS), testé par test/test_odometrie.py.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass
class ParametresProprio:
    gyro_bruit: float = 0.01        # rad/s, écart-type
    gyro_biais_max: float = 0.005   # rad/s, biais tiré une fois par voiture dans ±max
    acc_bruit: float = 0.05         # m/s², écart-type
    roue_echelle_max: float = 0.02  # erreur d'échelle de l'odomètre de roue, dans ±max
    roue_bruit: float = 0.01        # m/s, écart-type


class Proprioception:
    """IMU + odométrie d'UNE voiture. ``maj`` à chaque pas de simulation."""

    def __init__(self, rng: np.random.Generator, p: ParametresProprio = None):
        self.p = p or ParametresProprio()
        self.rng = rng
        self.biais = float(rng.uniform(-self.p.gyro_biais_max, self.p.gyro_biais_max))
        self.echelle = 1.0 + float(rng.uniform(-self.p.roue_echelle_max,
                                               self.p.roue_echelle_max))
        self.reinitialiser()

    def reinitialiser(self):
        """Nouvelle manche : l'odométrie repart de (0, 0, 0)."""
        self.x = self.y = self.cap = 0.0          # pose estimée (repère odom)
        self.v_roue = 0.0                         # vitesse mesurée à la roue (m/s)
        self.gyro_z = 0.0                         # lacet mesuré (rad/s)
        self.acc_x = self.acc_y = 0.0             # accélérations mesurées (m/s²)
        self._v = self._yaw = None

    def maj(self, v, yaw, dt):
        """Simulateur Python : vitesse longitudinale et cap RÉELS après un pas ``dt``."""
        if self._yaw is None or dt <= 0:
            self._v, self._yaw = v, yaw
            return
        lacet = math.atan2(math.sin(yaw - self._yaw), math.cos(yaw - self._yaw)) / dt
        self.mesurer(v, lacet, (v - self._v) / dt, v * lacet, dt)
        self._v, self._yaw = v, yaw

    def mesurer(self, v_roue, lacet, acc_x, acc_y, dt):
        """Grandeurs physiques (vitesse de roue, lacet, accélérations) -> mesures bruitées,
        puis intégration de l'odométrie. Utilisé tel quel par le driver Webots."""
        r = self.rng
        self.gyro_z = lacet + self.biais + float(r.normal(0.0, self.p.gyro_bruit))
        self.acc_x = acc_x + float(r.normal(0.0, self.p.acc_bruit))
        self.acc_y = acc_y + float(r.normal(0.0, self.p.acc_bruit))
        # roue arrêtée : le codeur ne compte rien (pas de bruit à l'arrêt)
        self.v_roue = 0.0 if abs(v_roue) < 1e-3 else \
            v_roue * self.echelle + float(r.normal(0.0, self.p.roue_bruit))
        # estime : cap intégré du gyroscope, position intégrée de la vitesse de roue
        cap_milieu = self.cap + 0.5 * self.gyro_z * dt
        self.x += self.v_roue * math.cos(cap_milieu) * dt
        self.y += self.v_roue * math.sin(cap_milieu) * dt
        self.cap = math.atan2(math.sin(self.cap + self.gyro_z * dt),
                              math.cos(self.cap + self.gyro_z * dt))
