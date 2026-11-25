"""
Monde simulé : circuit + voitures (modèle cinématique Ackermann) + capteurs.

Fonctions pures, sans ROS. Le nœud ``simulateur`` ne fait qu'appeler ``Monde.pas()``
et publier ``Monde.scan()`` / ``Monde.ultrasons()``.

Modèle « bicyclette » (repère REP 103, x avant, y gauche) :
    x' = v cos(yaw)    y' = v sin(yaw)    yaw' = v tan(braquage) / EMPATTEMENT
La vitesse suit la consigne avec une accélération bornée, le braquage avec une vitesse
de rotation bornée (servo), comme la TT02 de Webots. L'adhérence est limitée : si
v² · courbure dépasse ``adherence`` (m/s²), la courbure réelle est plafonnée et la voiture
élargit sa trajectoire. Aller vite en virage se paie.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from covapsy_simple_sim.carte import Circuit, Rectangle, ROUGE, VERT


@dataclass
class ParametresVehicule:
    longueur: float = 0.42          # m (TT02 + carrosserie)
    largeur: float = 0.19           # m
    empattement: float = 0.257      # m
    braquage_max: float = 0.35      # rad
    vitesse_max: float = 3.0        # m/s
    acceleration_max: float = 4.0   # m/s²
    freinage_max: float = 8.0       # m/s²
    vitesse_braquage: float = 4.0   # rad/s (servo)
    adherence: float = 6.0          # m/s², accélération latérale maximale (pneus) : au-delà,
    #                                 la voiture « tire tout droit » (sous-virage)
    lidar_x: float = 0.08           # m, position du lidar devant le centre (13 cm derrière
    #                                 le pare-chocs avant)
    timeout_commande: float = 0.5   # s, au-delà la consigne retombe à 0 (sécurité)


@dataclass
class ParametresCapteurs:
    lidar_points: int = 360
    lidar_portee: float = 12.0
    lidar_portee_min: float = 0.10
    lidar_bruit: float = 0.005      # m, écart-type
    lidar_perte: float = 0.0        # probabilité qu'un point soit invalide (inf)
    us_portee: float = 2.0
    us_portee_min: float = 0.02
    us_demi_cone: float = math.radians(15)
    camera_demi_champ: float = math.radians(35)   # champ horizontal de la caméra : ±35°
    camera_portee: float = 3.0                    # m, au-delà les bordures sont ignorées
    camera_colonnes: int = 64                     # « colonnes » d'image simulées


@dataclass
class Voiture:
    nom: str
    x: float
    y: float
    yaw: float
    v: float = 0.0
    braquage: float = 0.0
    consigne_v: float = 0.0
    consigne_braquage: float = 0.0
    t_consigne: float = -math.inf
    collisions: int = 0
    en_contact: bool = False
    contact_avec: str = ''
    dernier_contact: str = ''       # cible du dernier choc (conservée après la séparation)
    t_contact: float = -math.inf    # dernier instant en contact (anti-rebond du comptage)
    retiree: bool = False
    glisse: bool = False            # la voiture dépasse l'adhérence (sous-virage)
    distance: float = 0.0
    trajectoire: list = field(default_factory=list)

    def rectangle(self, p: ParametresVehicule):
        return Rectangle(self.x, self.y, self.yaw, p.longueur, p.largeur)


def _intersections_segments(ox, oy, angles, segments, portee):
    """Distance d'intersection rayon / segments (vectorisé). inf si rien."""
    if not segments:
        return np.full(angles.shape, np.inf)
    s = np.asarray(segments, dtype=float)           # [M, 2, 2]
    ax, ay = s[:, 0, 0] - ox, s[:, 0, 1] - oy
    ex, ey = s[:, 1, 0] - s[:, 0, 0], s[:, 1, 1] - s[:, 0, 1]
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    den = dx * ey - dy * ex                          # [N, M]
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (ax * ey - ay * ex) / den
        u = (ax * dy - ay * dx) / den
    ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1) & (t <= portee)
    t = np.where(ok, t, np.inf)
    return t.min(axis=1)


class Monde:

    def __init__(self, circuit: Circuit, noms, vehicule=None, capteurs=None, graine=0):
        self.circuit = circuit
        self.pv = vehicule or ParametresVehicule()
        self.pc = capteurs or ParametresCapteurs()
        self.rng = np.random.default_rng(graine)
        self.t = 0.0
        self.noms = list(noms)
        self.voitures = {}
        self.reinitialiser()

    # --- Cycle de vie ------------------------------------------------------------------
    def reinitialiser(self, poses=None):
        """Toutes les voitures sur la grille, à l'arrêt, compteurs remis à zéro.

        ``poses`` : {nom: (x, y, yaw)} imposées (rotation de la grille, sens inverse),
        envoyées par la direction de course ; sinon la grille du circuit dans l'ordre.
        """
        self.t = 0.0
        self.voitures = {}
        for i, nom in enumerate(self.noms):
            if poses and nom in poses:
                x, y, yaw = poses[nom]
            else:
                p = self.circuit.grille[i % len(self.circuit.grille)]
                x, y, yaw = p.x, p.y, p.yaw
            self.voitures[nom] = Voiture(nom, float(x), float(y), float(yaw))

    def retirer(self, nom):
        """Retire une voiture de la piste : plus d'interaction physique ni de mouvement."""
        v = self.voitures[nom]
        v.retiree, v.v, v.consigne_v = True, 0.0, 0.0

    def commander(self, nom, vitesse, braquage):
        v = self.voitures[nom]
        v.consigne_v = float(np.clip(vitesse, -self.pv.vitesse_max, self.pv.vitesse_max))
        v.consigne_braquage = float(np.clip(braquage, -self.pv.braquage_max,
                                            self.pv.braquage_max))
        v.t_consigne = self.t

    # --- Dynamique ---------------------------------------------------------------------
    def pas(self, dt):
        self.t += dt
        p = self.pv
        for v in self.voitures.values():
            if v.retiree:
                continue
            cible_v, cible_b = v.consigne_v, v.consigne_braquage
            if self.t - v.t_consigne > p.timeout_commande:      # plus de commande : arrêt
                cible_v = 0.0
            acc = p.acceleration_max if abs(cible_v) > abs(v.v) else p.freinage_max
            v.v += float(np.clip(cible_v - v.v, -acc * dt, acc * dt))
            db = p.vitesse_braquage * dt
            v.braquage += float(np.clip(cible_b - v.braquage, -db, db))
            courbure = math.tan(v.braquage) / p.empattement
            if v.v * v.v * abs(courbure) > p.adherence:     # trop vite pour ce virage :
                courbure = math.copysign(p.adherence / (v.v * v.v), courbure)   # sous-virage
                v.glisse = True
            else:
                v.glisse = False
            yaw = v.yaw + v.v * courbure * dt
            x = v.x + v.v * math.cos(v.yaw) * dt
            y = v.y + v.v * math.sin(v.yaw) * dt
            autre = self._collision(v, x, y, yaw)
            if not autre:
                v.distance += math.hypot(x - v.x, y - v.y)
                v.x, v.y, v.yaw = x, y, math.atan2(math.sin(yaw), math.cos(yaw))
                v.en_contact, v.contact_avec = False, ''
                continue
            if not v.en_contact and self.t - v.t_contact > 0.5:
                v.collisions += 1               # un nouveau choc, pas une voiture qui pousse
                v.dernier_contact = autre
            v.t_contact = self.t
            v.en_contact, v.contact_avec = True, autre
            # Réponse au choc : glisser le long de l'obstacle (translation seule, vitesse
            # divisée par 2), sinon pivoter sur place, sinon s'arrêter. Évite que deux
            # voitures en contact latéral restent bloquées indéfiniment.
            if not self._collision(v, x, y, v.yaw):
                v.distance += math.hypot(x - v.x, y - v.y)
                v.x, v.y, v.v = x, y, 0.5 * v.v
            elif yaw != v.yaw and not self._collision(v, v.x, v.y, yaw):
                v.yaw, v.v = math.atan2(math.sin(yaw), math.cos(yaw)), 0.0
            else:
                v.v = 0.0

    def _points_contour(self, x, y, yaw, n=6):
        p = self.pv
        c, s = math.cos(yaw), math.sin(yaw)
        hl, hw = p.longueur / 2, p.largeur / 2
        u = np.linspace(-1, 1, n)
        loc = np.vstack([np.c_[u * hl, np.full(n, hw)], np.c_[u * hl, np.full(n, -hw)],
                         np.c_[np.full(n, hl), u * hw], np.c_[np.full(n, -hl), u * hw]])
        return x + c * loc[:, 0] - s * loc[:, 1], y + s * loc[:, 0] + c * loc[:, 1]

    def _collision(self, v, x, y, yaw):
        px, py = self._points_contour(x, y, yaw)
        if self.circuit.occupe(px, py).any():
            return 'bordure'
        for o in self.rectangles_dynamiques(exclure=v.nom):
            r, nom = o
            c, s = math.cos(r.yaw), math.sin(r.yaw)
            lx = c * (px - r.x) + s * (py - r.y)
            ly = -s * (px - r.x) + c * (py - r.y)
            if np.any((np.abs(lx) <= r.longueur / 2) & (np.abs(ly) <= r.largeur / 2)):
                return nom
        return ''

    def rectangles_dynamiques(self, exclure=None):
        """Voitures (non retirées) et obstacles mobiles, sous forme de rectangles."""
        out = [(w.rectangle(self.pv), w.nom) for w in self.voitures.values()
               if w.nom != exclure and not w.retiree]
        out += [(o.rectangle(self.t), 'obstacle_mobile') for o in self.circuit.obstacles_mobiles]
        return out

    # --- Capteurs ----------------------------------------------------------------------
    def scan(self, nom):
        """Scan lidar de la voiture ``nom`` : (angles relatifs croissants, distances)."""
        v, pc = self.voitures[nom], self.pc
        a_rel = np.linspace(-math.pi, math.pi, pc.lidar_points, endpoint=False)
        ox = v.x + self.pv.lidar_x * math.cos(v.yaw)
        oy = v.y + self.pv.lidar_x * math.sin(v.yaw)
        a = a_rel + v.yaw
        d = self.circuit.lancer_rayons(ox, oy, a, pc.lidar_portee)
        segs = [s for r, _ in self.rectangles_dynamiques(exclure=nom) for s in r.segments()]
        d = np.minimum(d, _intersections_segments(ox, oy, a, segs, pc.lidar_portee))
        if pc.lidar_bruit > 0:
            d = d + self.rng.normal(0.0, pc.lidar_bruit, d.shape)
        if pc.lidar_perte > 0:
            d[self.rng.random(d.shape) < pc.lidar_perte] = np.inf
        d[d < pc.lidar_portee_min] = np.inf
        return a_rel, d

    def ultrasons(self, nom):
        """Distances des 4 ultrasons (avant, arriere, gauche, droite), mesurées au bord."""
        v, pv, pc = self.voitures[nom], self.pv, self.pc
        out = {}
        for cote, ang, off in (('avant', 0.0, pv.longueur / 2), ('arriere', math.pi, pv.longueur / 2),
                               ('gauche', math.pi / 2, pv.largeur / 2),
                               ('droite', -math.pi / 2, pv.largeur / 2)):
            a0 = v.yaw + ang
            ox, oy = v.x + off * math.cos(a0), v.y + off * math.sin(a0)
            a = a0 + np.linspace(-pc.us_demi_cone, pc.us_demi_cone, 5)
            d = self.circuit.lancer_rayons(ox, oy, a, pc.us_portee, pas=0.01)
            segs = [s for r, _ in self.rectangles_dynamiques(exclure=nom) for s in r.segments()]
            d = np.minimum(d, _intersections_segments(ox, oy, a, segs, pc.us_portee))
            m = float(d.min())
            out[cote] = m if pc.us_portee_min <= m <= pc.us_portee else math.inf
        return out

    def couleurs_murs(self, nom):
        """Caméra simulée : proportions de bordures vertes / rouges vues à gauche et à droite.

        Comme ``color_utils.color_fractions`` sous Webots : l'image est coupée en deux
        moitiés (gauche : angles > 0) ; pour chaque moitié, part des colonnes qui voient une
        bordure verte, rouge. Une voiture devant masque les murs.
        """
        v, pc, c = self.voitures[nom], self.pc, self.circuit
        a_rel = np.linspace(pc.camera_demi_champ, -pc.camera_demi_champ, pc.camera_colonnes)
        ox = v.x + self.pv.lidar_x * math.cos(v.yaw)
        oy = v.y + self.pv.lidar_x * math.sin(v.yaw)
        a = a_rel + v.yaw
        d = c.lancer_rayons(ox, oy, a, pc.camera_portee)
        segs = [s for r, _ in self.rectangles_dynamiques(exclure=nom) for s in r.segments()]
        masque = _intersections_segments(ox, oy, a, segs, pc.camera_portee) < d
        vu = np.isfinite(d) & ~masque
        dd = np.where(vu, d, 0.0) + 2 * c.resolution   # entrer dans le mur pour lire sa couleur
        col = c.couleur_en(ox + dd * np.cos(a), oy + dd * np.sin(a))
        col[~vu] = 0
        g, dr = a_rel > 0, a_rel < 0
        return {'gauche_vert': float(np.mean(col[g] == VERT)),
                'gauche_rouge': float(np.mean(col[g] == ROUGE)),
                'droite_vert': float(np.mean(col[dr] == VERT)),
                'droite_rouge': float(np.mean(col[dr] == ROUGE))}

    # --- Vérité terrain (réservée à l'enseignant / l'arbitre) -------------------------
    def verite_terrain(self):
        return {'t': round(self.t, 4), 'voitures': {
            n: {'x': round(v.x, 4), 'y': round(v.y, 4), 'yaw': round(v.yaw, 4),
                'v': round(v.v, 3), 'braquage': round(v.braquage, 3),
                'collisions': v.collisions, 'contact': v.contact_avec or v.dernier_contact,
                'retiree': v.retiree,
                'distance': round(v.distance, 3)}
            for n, v in self.voitures.items()},
            'obstacles_mobiles': [
                {'x': round(r.x, 3), 'y': round(r.y, 3), 'yaw': round(r.yaw, 3),
                 'longueur': r.longueur, 'largeur': r.largeur}
                for r in (o.rectangle(self.t) for o in self.circuit.obstacles_mobiles)]}
