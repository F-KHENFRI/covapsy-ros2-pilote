"""
Circuit 2D : grille d'occupation, ligne centrale, grille de départ, ligne d'arrivée.

Un circuit est décrit par un fichier YAML (dossier ``circuits/``) de deux façons :

  1. ``ligne_centrale`` + ``largeur`` : la piste est générée (bordures à ± largeur/2) ;
  2. ``carte: fichier.npz`` : grille d'occupation précalculée (ex. piste officielle
     convertie depuis Webots par ``tools/wbt_vers_carte.py``).

La ``ligne_centrale`` (boucle fermée, dans le SENS DE COURSE) sert aussi à l'arbitre :
abscisse curviligne, progression, détection du contresens. Son premier point est sur la
ligne d'arrivée.

Fonctions pures, sans ROS : utilisables dans les tests et par l'interface web.
"""
from dataclasses import dataclass, field, replace
import math
import os

import numpy as np
import yaml

# Couleurs des bordures (règlement CoVAPSy : vert d'un côté, rouge de l'autre)
AUCUNE, VERT, ROUGE = 0, 1, 2
COULEURS = {'vert': VERT, 'rouge': ROUGE, 'green': VERT, 'red': ROUGE}
NOMS_COULEURS = {AUCUNE: '', VERT: 'vert', ROUGE: 'rouge'}
EPAISSEUR_BORDURE = 4                 # cellules colorées le long de la piste


def _dossier_circuits():
    """Dossier des circuits : sources (essais, --symlink-install) ou share/ du paquet installé."""
    ici = os.path.join(os.path.dirname(__file__), 'circuits')
    if os.path.isdir(ici):
        return ici
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('covapsy_simple_sim'), 'circuits')


DOSSIER_CIRCUITS = _dossier_circuits()


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


@dataclass
class Rectangle:
    """Obstacle rectangulaire (centre, orientation, dimensions)."""

    x: float
    y: float
    yaw: float = 0.0
    longueur: float = 0.45
    largeur: float = 0.20

    def coins(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hl, hw = self.longueur / 2, self.largeur / 2
        return [(self.x + c * dx - s * dy, self.y + s * dx + c * dy)
                for dx, dy in ((hl, hw), (-hl, hw), (-hl, -hw), (hl, -hw))]

    def segments(self):
        p = self.coins()
        return [(p[i], p[(i + 1) % 4]) for i in range(4)]


@dataclass
class ObstacleMobile:
    """Obstacle en va-et-vient entre deux points (période en s)."""

    x0: float
    y0: float
    x1: float
    y1: float
    periode: float = 8.0
    longueur: float = 0.30
    largeur: float = 0.20

    def rectangle(self, t):
        a = 0.5 - 0.5 * math.cos(2 * math.pi * t / self.periode)
        return Rectangle(self.x0 + a * (self.x1 - self.x0), self.y0 + a * (self.y1 - self.y0),
                         math.atan2(self.y1 - self.y0, self.x1 - self.x0),
                         self.longueur, self.largeur)


@dataclass
class Circuit:
    nom: str
    grille_occ: np.ndarray            # bool [ny, nx], True = mur / obstacle fixe
    origine: tuple                    # (x, y) du coin bas-gauche de la cellule [0, 0]
    resolution: float                 # m / cellule
    ligne_centrale: np.ndarray        # [N, 2], boucle fermée (dernier point != premier)
    grille: list                      # [Pose] grille de départ
    ligne_arrivee: tuple              # ((x1, y1), (x2, y2))
    obstacles_mobiles: list = field(default_factory=list)
    description: str = ''
    grille_murs: np.ndarray = None    # murs SANS les obstacles (bordures colorées)
    couleur_droite: str = 'vert'      # couleur du mur de DROITE dans le sens de course
    sens: str = 'normal'              # normal | inverse (voir inverse())
    grille_inverse: list = None       # [Pose] grille du sens inverse (facultatif)

    # --- Géométrie de la ligne centrale --------------------------------------------
    def __post_init__(self):
        lc = np.asarray(self.ligne_centrale, dtype=float)
        seg = np.roll(lc, -1, axis=0) - lc
        self._lc, self._seg = lc, seg
        self._len = np.hypot(seg[:, 0], seg[:, 1])
        self._s0 = np.concatenate(([0.0], np.cumsum(self._len)[:-1]))
        self.longueur = float(self._len.sum())

    def abscisse(self, x, y, s_prec=None, fenetre=2.5):
        """
        (s, distance latérale, cap de la piste) du point projeté sur la ligne centrale.

        ``s_prec`` : abscisse précédente du même véhicule. La recherche est alors limitée
        à ± ``fenetre`` m autour d'elle, pour ne pas sauter dans un couloir voisin
        (deux portions de piste séparées par une simple bordure).
        """
        p = np.array([x, y])
        d = p - self._lc
        u = np.clip(np.einsum('ij,ij->i', d, self._seg) / np.maximum(self._len ** 2, 1e-12),
                    0.0, 1.0)
        proj = self._lc + u[:, None] * self._seg
        dist = np.hypot(*(p - proj).T)
        if s_prec is not None:
            ecart = np.abs((self._s0 + u * self._len - s_prec + self.longueur / 2)
                           % self.longueur - self.longueur / 2)
            dist = np.where(ecart <= fenetre, dist, np.inf)
            if not np.isfinite(dist).any():
                return self.abscisse(x, y)
        i = int(np.argmin(dist))
        cap = math.atan2(self._seg[i, 1], self._seg[i, 0])
        return float(self._s0[i] + u[i] * self._len[i]), float(dist[i]), cap

    def point(self, s):
        """(x, y, cap) du point de la ligne centrale à l'abscisse s (m, modulo la longueur)."""
        s = s % self.longueur
        i = int(np.searchsorted(self._s0, s, side='right') - 1)
        u = (s - self._s0[i]) / max(self._len[i], 1e-12)
        x, y = self._lc[i] + u * self._seg[i]
        return float(x), float(y), math.atan2(self._seg[i, 1], self._seg[i, 0])

    def lateral(self, x, y):
        """(s, écart latéral signé : > 0 à GAUCHE de la ligne centrale dans le sens de course)."""
        s, _, _ = self.abscisse(x, y)
        cx, cy, cap = self.point(s)
        return s, -(x - cx) * math.sin(cap) + (y - cy) * math.cos(cap)

    def inverse(self):
        """Le même circuit parcouru dans l'autre sens (manches « sens inverse »).

        Ligne centrale retournée, ligne d'arrivée inchangée. Grille : ``grille_inverse``
        du YAML si elle existe, sinon chaque place est symétrique de la place d'origine par
        rapport à la ligne d'arrivée (d m derrière la ligne -> d m de l'autre côté, nez vers
        la ligne). Les bordures gardent leurs couleurs physiques : la couleur de droite
        (dans le sens de course) s'inverse donc.
        """
        if self.grille_inverse:
            grille = list(self.grille_inverse)
        else:
            (ax_, ay_), (bx_, by_) = self.ligne_arrivee
            th = math.atan2(by_ - ay_, bx_ - ax_)            # direction de la ligne
            ux, uy = math.cos(th), math.sin(th)
            grille = []
            for p in self.grille:
                dx, dy = p.x - ax_, p.y - ay_
                k = dx * ux + dy * uy                        # symétrie axiale / la ligne
                grille.append(Pose(ax_ + 2 * k * ux - dx, ay_ + 2 * k * uy - dy,
                                   math.atan2(math.sin(2 * th - p.yaw), math.cos(2 * th - p.yaw))))
        lc = self._lc[::-1].copy()
        (ax_, ay_), (bx_, by_) = self.ligne_arrivee
        milieu = np.array([(ax_ + bx_) / 2, (ay_ + by_) / 2])
        lc = np.roll(lc, -int(np.argmin(np.hypot(*(lc - milieu).T))), axis=0)
        autre = {'vert': 'rouge', 'green': 'rouge', 'rouge': 'vert', 'red': 'vert'}
        c = replace(self, ligne_centrale=lc, grille=grille, grille_inverse=list(self.grille),
                    couleur_droite=autre[self.couleur_droite],
                    sens='inverse' if self.sens == 'normal' else 'normal')
        c._couleurs = self.couleurs()                # couleurs physiques : inchangées
        c._edt = getattr(self, '_edt', None)
        return c

    # --- Grille d'occupation ---------------------------------------------------------
    def occupe(self, x, y):
        """Vrai si (x, y) est dans un mur (tableaux NumPy acceptés). Hors carte = mur."""
        ix = np.floor((np.asarray(x) - self.origine[0]) / self.resolution).astype(int)
        iy = np.floor((np.asarray(y) - self.origine[1]) / self.resolution).astype(int)
        ny, nx = self.grille_occ.shape
        dedans = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        out = np.ones(np.shape(ix), dtype=bool)
        out[dedans] = self.grille_occ[iy[dedans], ix[dedans]]
        return out

    def distance_murs(self):
        """Carte des distances au mur le plus proche (m), calculée une fois (cache)."""
        if getattr(self, '_edt', None) is None:
            from scipy.ndimage import distance_transform_edt
            self._edt = distance_transform_edt(~self.grille_occ) * self.resolution
        return self._edt

    def lancer_rayons(self, x, y, angles, portee, pas=None):
        """
        Distance au premier mur fixe le long de chaque rayon (inf si rien).

        « Sphere tracing » : on avance chaque rayon de la distance au mur le plus proche
        (carte précalculée), d'où une vingtaine d'itérations au lieu de centaines de pas.
        """
        edt = self.distance_murs()
        ny, nx = edt.shape
        res = self.resolution
        dx, dy = np.cos(angles), np.sin(angles)
        t = np.zeros(len(angles))
        actif = np.ones(len(angles), dtype=bool)
        seuil = res * 0.75
        for _ in range(200):
            if not actif.any():
                break
            ix = np.floor((x + dx[actif] * t[actif] - self.origine[0]) / res).astype(int)
            iy = np.floor((y + dy[actif] * t[actif] - self.origine[1]) / res).astype(int)
            dedans = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            d = np.zeros(ix.shape)
            d[dedans] = edt[iy[dedans], ix[dedans]]
            idx = np.flatnonzero(actif)
            touche = d < seuil
            t[idx] += np.where(touche, 0.0, np.maximum(d - res * 0.5, res * 0.5))
            sortis = t[idx] > portee
            actif[idx[touche | sortis]] = False
        out = t.copy()
        out[t > portee] = np.inf
        return out

    def couleurs(self):
        """Couleur des bordures, int8 [ny, nx] : AUCUNE, VERT ou ROUGE (calculée une fois).

        Règlement CoVAPSy : une couleur par côté de la piste. Une cellule de bordure prend
        la couleur du côté de la ligne centrale (sens de course) où elle se trouve : à droite
        ``couleur_droite``, à gauche l'autre. Les obstacles restent sans couleur.
        """
        if getattr(self, '_couleurs', None) is None:
            from scipy.ndimage import distance_transform_edt
            from scipy.spatial import cKDTree
            murs = self.grille_occ if self.grille_murs is None else self.grille_murs
            # pour chaque mur : distance (cellules) et indices de la cellule libre la plus proche
            prof, (jy, jx) = distance_transform_edt(murs, return_indices=True)
            bord = murs & (prof <= EPAISSEUR_BORDURE)
            iy, ix = np.nonzero(bord)
            fx = self.origine[0] + (jx[iy, ix] + 0.5) * self.resolution
            fy = self.origine[1] + (jy[iy, ix] + 0.5) * self.resolution
            _, k = cKDTree(self._lc).query(np.c_[fx, fy])
            tx, ty = (self._seg[k] / self._len[k, None]).T
            # > 0 : le mur est à GAUCHE de la ligne centrale (produit vectoriel)
            px = self.origine[0] + (ix + 0.5) * self.resolution
            py = self.origine[1] + (iy + 0.5) * self.resolution
            gauche = tx * (py - self._lc[k, 1]) - ty * (px - self._lc[k, 0]) > 0
            droite = COULEURS[self.couleur_droite]
            c = np.full(murs.shape, AUCUNE, dtype=np.int8)
            c[iy, ix] = np.where(gauche, VERT + ROUGE - droite, droite)
            self._couleurs = c
        return self._couleurs

    def couleur_en(self, x, y):
        """Couleur de bordure aux points (x, y) (AUCUNE hors carte)."""
        c = self.couleurs()
        ix = np.floor((np.asarray(x) - self.origine[0]) / self.resolution).astype(int)
        iy = np.floor((np.asarray(y) - self.origine[1]) / self.resolution).astype(int)
        ny, nx = c.shape
        dedans = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        out = np.full(np.shape(ix), AUCUNE, dtype=np.int8)
        out[dedans] = c[iy[dedans], ix[dedans]]
        return out

    def limites(self):
        ny, nx = self.grille_occ.shape
        x0, y0 = self.origine
        return x0, y0, x0 + nx * self.resolution, y0 + ny * self.resolution


def franchit(p0, p1, ligne):
    """+1 / -1 si le segment p0 -> p1 coupe la ligne (gauche->droite / inverse), 0 sinon."""
    (ax, ay), (bx, by) = ligne

    def cote(px, py):
        return (bx - ax) * (py - ay) - (by - ay) * (px - ax)

    c0, c1 = cote(*p0), cote(*p1)
    if c0 * c1 >= 0:
        return 0
    d0 = (p1[1] - p0[1]) * (ax - p0[0]) - (p1[0] - p0[0]) * (ay - p0[1])
    d1 = (p1[1] - p0[1]) * (bx - p0[0]) - (p1[0] - p0[0]) * (by - p0[1])
    if d0 * d1 > 0:
        return 0
    return 1 if c0 > 0 else -1


# -----------------------------------------------------------------------------------
# Rasterisation
# -----------------------------------------------------------------------------------
def _grille_vide(xmin, ymin, xmax, ymax, res):
    nx, ny = int(math.ceil((xmax - xmin) / res)), int(math.ceil((ymax - ymin) / res))
    return np.zeros((ny, nx), dtype=bool), (xmin, ymin)


def _centres(grille, origine, res):
    ny, nx = grille.shape
    xs = origine[0] + (np.arange(nx) + 0.5) * res
    ys = origine[1] + (np.arange(ny) + 0.5) * res
    return np.meshgrid(xs, ys)


def remplir_polygone(grille, origine, res, poly):
    """Marque occupées les cellules dont le centre est dans le polygone ``poly``."""
    from matplotlib.path import Path
    poly = np.asarray(poly, dtype=float)
    ny, nx = grille.shape
    i0 = max(0, int((poly[:, 0].min() - origine[0]) / res) - 1)
    i1 = min(nx, int((poly[:, 0].max() - origine[0]) / res) + 2)
    j0 = max(0, int((poly[:, 1].min() - origine[1]) / res) - 1)
    j1 = min(ny, int((poly[:, 1].max() - origine[1]) / res) + 2)
    if i0 >= i1 or j0 >= j1:
        return
    xs = origine[0] + (np.arange(i0, i1) + 0.5) * res
    ys = origine[1] + (np.arange(j0, j1) + 0.5) * res
    gx, gy = np.meshgrid(xs, ys)
    m = Path(poly).contains_points(np.c_[gx.ravel(), gy.ravel()], radius=res * 0.5)
    grille[j0:j1, i0:i1] |= m.reshape(gx.shape)


def generer_depuis_ligne_centrale(lc, largeur, res, marge=1.0):
    """Piste de ``largeur`` m autour d'une ligne centrale fermée : tout est mur sauf la piste."""
    lc = np.asarray(lc, dtype=float)
    xmin, ymin = lc.min(axis=0) - largeur / 2 - marge
    xmax, ymax = lc.max(axis=0) + largeur / 2 + marge
    grille, origine = _grille_vide(xmin, ymin, xmax, ymax, res)
    gx, gy = _centres(grille, origine, res)
    pts = np.c_[gx.ravel(), gy.ravel()]
    # distance de chaque cellule à la polyligne fermée
    a = lc
    b = np.roll(lc, -1, axis=0)
    dmin = np.full(pts.shape[0], np.inf)
    for (ax, ay), (bx, by) in zip(a, b):
        abx, aby = bx - ax, by - ay
        L2 = abx * abx + aby * aby or 1e-12
        u = np.clip(((pts[:, 0] - ax) * abx + (pts[:, 1] - ay) * aby) / L2, 0.0, 1.0)
        dmin = np.minimum(dmin, np.hypot(pts[:, 0] - ax - u * abx, pts[:, 1] - ay - u * aby))
    grille[:] = (dmin > largeur / 2).reshape(grille.shape)
    return grille, origine


def lisser_boucle(points, pas=0.1):
    """Rééchantillonne une boucle fermée à pas régulier (interpolation linéaire)."""
    p = np.asarray(points, dtype=float)
    q = np.vstack([p, p[:1]])
    d = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(q, axis=0).T))))
    s = np.arange(0.0, d[-1], pas)
    return np.c_[np.interp(s, d, q[:, 0]), np.interp(s, d, q[:, 1])]


def polygone_arrondi(coins, rayon, pas=0.05):
    """Boucle passant par des coins arrondis (rayon en m), dans l'ordre donné (sens de course)."""
    c = np.asarray(coins, dtype=float)
    n = len(c)
    pts = []
    for i in range(n):
        p0, p1, p2 = c[i - 1], c[i], c[(i + 1) % n]
        u = (p0 - p1) / np.linalg.norm(p0 - p1)
        w = (p2 - p1) / np.linalg.norm(p2 - p1)
        demi = math.acos(float(np.clip(np.dot(u, w), -1.0, 1.0))) / 2
        r = min(rayon, 0.45 * min(np.linalg.norm(p0 - p1), np.linalg.norm(p2 - p1))
                * math.tan(demi))
        d = r / math.tan(demi)
        a, b = p1 + u * d, p1 + w * d                  # points de tangence
        bis = (u + w) / np.linalg.norm(u + w)
        centre = p1 + bis * r / math.sin(demi)
        t0 = math.atan2(*(a - centre)[::-1])
        t1 = math.atan2(*(b - centre)[::-1])
        dt = math.atan2(math.sin(t1 - t0), math.cos(t1 - t0))
        k = max(2, int(abs(dt) * r / pas))
        pts += [centre + r * np.array([math.cos(t0 + dt * j / k), math.sin(t0 + dt * j / k)])
                for j in range(k + 1)]
    return lisser_boucle(np.array(pts), pas)


def generer_forme(f):
    """Ligne centrale d'une forme paramétrique : {type: ovale|polygone, ...}."""
    if f['type'] == 'ovale':
        L, R = float(f['longueur_droite']), float(f['rayon'])
        return polygone_arrondi([[L / 2 + R, -R], [L / 2 + R, R], [-L / 2 - R, R],
                                 [-L / 2 - R, -R]], R)
    if f['type'] == 'polygone':
        return polygone_arrondi(f['coins'], float(f.get('rayon', 0.6)))
    raise ValueError(f"forme inconnue : {f['type']}")


# -----------------------------------------------------------------------------------
# Chargement
# -----------------------------------------------------------------------------------
def chemin_circuit(nom):
    """Accepte un nom ('ovale'), un fichier ('ovale.yaml') ou un chemin complet."""
    if os.path.isfile(nom):
        return nom
    base = nom if nom.endswith('.yaml') else nom + '.yaml'
    chemin = os.path.join(DOSSIER_CIRCUITS, base)
    if not os.path.isfile(chemin):
        dispo = ', '.join(sorted(f[:-5] for f in os.listdir(DOSSIER_CIRCUITS)
                                 if f.endswith('.yaml')))
        raise FileNotFoundError(f'circuit « {nom} » introuvable (disponibles : {dispo})')
    return chemin


def lister_circuits():
    return sorted(f[:-5] for f in os.listdir(DOSSIER_CIRCUITS) if f.endswith('.yaml'))


def charger_circuit(nom, obstacles=True, couleur_droite=None):
    """Charge un circuit YAML. ``obstacles=False`` ignore les obstacles fixes et mobiles.

    ``couleur_droite`` (vert ou rouge) : couleur du mur de droite dans le sens de course ;
    par défaut celle du YAML (clé ``couleur_droite``), sinon vert.
    """
    chemin = chemin_circuit(nom)
    with open(chemin) as f:
        d = yaml.safe_load(f)
    res = float(d.get('resolution', 0.02))
    lc = generer_forme(d['forme']) if 'forme' in d else np.asarray(d['ligne_centrale'],
                                                                   dtype=float)
    if d.get('lisser_pas'):
        lc = lisser_boucle(lc, float(d['lisser_pas']))
    if 'carte' in d:
        z = np.load(os.path.join(os.path.dirname(chemin), d['carte']))
        grille, origine, res = z['grille'].astype(bool), tuple(z['origine']), float(z['resolution'])
    else:
        grille, origine = generer_depuis_ligne_centrale(lc, float(d['largeur']), res)
    grille_murs = grille.copy()
    if obstacles:
        for o in d.get('obstacles', []) or []:
            remplir_polygone(grille, origine, res, Rectangle(**o).coins())
    couleur_droite = couleur_droite or d.get('couleur_droite', 'vert')
    if couleur_droite not in COULEURS:
        raise ValueError(f'couleur_droite « {couleur_droite} » : vert ou rouge attendu')
    grille_dep = [Pose(**p) for p in d['grille']]
    grille_inv = [Pose(**p) for p in d['grille_inverse']] if d.get('grille_inverse') else None
    la = d['ligne_arrivee']
    # la ligne centrale commence sur la ligne d'arrivée (abscisse s = 0)
    milieu = (np.asarray(la[0], dtype=float) + np.asarray(la[1], dtype=float)) / 2
    lc = np.roll(lc, -int(np.argmin(np.hypot(*(lc - milieu).T))), axis=0)
    mobiles = [ObstacleMobile(**o) for o in (d.get('obstacles_mobiles') or [])] \
        if obstacles else []
    return Circuit(nom=d.get('nom', os.path.basename(chemin)[:-5]), grille_occ=grille,
                   origine=origine, resolution=res, ligne_centrale=lc, grille=grille_dep,
                   ligne_arrivee=((la[0][0], la[0][1]), (la[1][0], la[1][1])),
                   obstacles_mobiles=mobiles, description=d.get('description', ''),
                   grille_murs=grille_murs, grille_inverse=grille_inv,
                   couleur_droite=couleur_droite)
