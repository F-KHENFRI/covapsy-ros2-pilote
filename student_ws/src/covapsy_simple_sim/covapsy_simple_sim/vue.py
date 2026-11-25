"""Fenêtre matplotlib du simulateur : piste, voitures, points lidar, trajectoire, chronos,
et direction visée par chaque pilote (flèche, topic pilote/angle_cible)."""
import math
import time

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

COULEURS = ['#e6b800', '#8e24aa', '#1e5bff', '#ff6d00', '#00a86b', '#d81b60']
LONGUEUR_FLECHE = 1.0       # m : flèche « direction visée » (pilote/angle_cible)


class Vue:

    def __init__(self, circuit, monde, tours, periode=0.1, cibles=None):
        matplotlib.rcParams['toolbar'] = 'None'
        self.monde, self.tours, self.periode = monde, tours, periode
        self.cibles = cibles if cibles is not None else {}   # nom -> (angle, instant)
        self.t_aff = 0.0
        plt.ion()
        # panneau des chronos SOUS la piste (hors du tracé) : 1 ligne par voiture + le temps
        n_lignes = 1 + len(monde.voitures)
        panneau = 0.2 * n_lignes + 0.2                      # hauteur en pouces
        self.fig, self.ax = plt.subplots(figsize=(8, 7 + panneau))
        self.fig.subplots_adjust(bottom=(panneau + 0.35) / (7 + panneau), top=0.95)
        self.fig.canvas.manager.set_window_title('CoVAPSy — simulateur Python')
        x0, y0, x1, y1 = circuit.limites()
        self.ax.imshow(circuit.grille_occ, origin='lower', extent=[x0, x1, y0, y1],
                       cmap='Greys', vmin=0, vmax=1.4, interpolation='nearest')
        # bordures colorées (vert / rouge) : ce que voit la caméra
        from covapsy_simple_sim.carte import ROUGE, VERT
        coul = circuit.couleurs()
        rgba = np.zeros(coul.shape + (4,))
        rgba[coul == VERT] = (0.0, 0.66, 0.2, 1.0)
        rgba[coul == ROUGE] = (0.85, 0.1, 0.1, 1.0)
        self.ax.imshow(rgba, origin='lower', extent=[x0, x1, y0, y1], interpolation='nearest')
        (ax_, ay_), (bx_, by_) = circuit.ligne_arrivee
        self.ax.plot([ax_, bx_], [ay_, by_], color='white', lw=3)
        self.ax.plot([ax_, bx_], [ay_, by_], color='black', lw=1, ls=':')
        self.ax.set_aspect('equal')
        self.ax.set_xlim(x0, x1)
        self.ax.set_ylim(y0, y1)
        self.ax.set_title(f'circuit « {circuit.nom} »')
        self.corps, self.traces, self.hist, self.fleches = {}, {}, {}, {}
        for i, nom in enumerate(monde.voitures):
            c = COULEURS[i % len(COULEURS)]
            self.corps[nom] = self.ax.add_patch(Polygon(np.zeros((4, 2)), closed=True, fc=c,
                                                        ec='black', zorder=5))
            self.traces[nom], = self.ax.plot([], [], color=c, lw=1, alpha=0.7)
            # direction visée par le pilote (topic pilote/angle_cible) : trait + pointe
            self.fleches[nom], = self.ax.plot([], [], color=c, lw=2.5, marker='o', ms=5,
                                              markevery=[1], mec='black', zorder=6)
            self.hist[nom] = []
        self.lidar, = self.ax.plot([], [], '.', color='red', ms=2, zorder=4)
        self.mobiles = []
        hauteur = self.fig.get_figheight()
        self.textes = [self.fig.text(0.03, (panneau - 0.2 * i) / hauteur, '', va='top',
                                     family='monospace', fontsize=9, fontweight='bold',
                                     color='black' if i == 0 else
                                     COULEURS[(i - 1) % len(COULEURS)])
                       for i in range(n_lignes)]
        plt.show(block=False)

    def maj(self, t):
        """Redessine (au plus tous les ``periode`` s de temps réel). False si fenêtre fermée."""
        if not plt.fignum_exists(self.fig.number):
            return False
        if time.monotonic() - self.t_aff < self.periode:
            return True
        self.t_aff = time.monotonic()
        m = self.monde
        lignes = [f't = {t:7.1f} s']
        for nom, v in m.voitures.items():
            self.corps[nom].set_xy(v.rectangle(m.pv).coins())
            self.hist[nom].append((v.x, v.y))
            self.hist[nom] = self.hist[nom][-600:]
            h = np.array(self.hist[nom])
            self.traces[nom].set_data(h[:, 0], h[:, 1])
            k = self.tours.get(nom, {})
            fmt = (lambda x: f'{x:6.2f}' if x else '     -')       # noqa: E731
            lignes.append(f'{nom}: tours {max(0, k.get("n", 0)):2d}  dernier {fmt(k.get("dernier"))}'
                          f'  meilleur {fmt(k.get("meilleur"))}  v {v.v:+.2f}  chocs {v.collisions}'
                          + ('  RETIRÉE' if v.retiree else ''))
        for nom, v in m.voitures.items():
            cible = self.cibles.get(nom)
            if cible is None or v.retiree or time.monotonic() - cible[1] > 1.0:
                self.fleches[nom].set_data([], [])
                continue
            ox, oy = v.x + m.pv.lidar_x * math.cos(v.yaw), v.y + m.pv.lidar_x * math.sin(v.yaw)
            a = v.yaw + cible[0]
            self.fleches[nom].set_data([ox, ox + LONGUEUR_FLECHE * math.cos(a)],
                                       [oy, oy + LONGUEUR_FLECHE * math.sin(a)])
        premiere = next(iter(m.voitures))
        a, d = m.scan(premiere)
        v = m.voitures[premiere]
        ok = np.isfinite(d)
        ox, oy = v.x + m.pv.lidar_x * math.cos(v.yaw), v.y + m.pv.lidar_x * math.sin(v.yaw)
        self.lidar.set_data(ox + d[ok] * np.cos(a[ok] + v.yaw), oy + d[ok] * np.sin(a[ok] + v.yaw))
        for p in self.mobiles:
            p.remove()
        self.mobiles = [self.ax.add_patch(Polygon(o.rectangle(t).coins(), fc='#555', zorder=4))
                        for o in m.circuit.obstacles_mobiles]
        for texte, ligne in zip(self.textes, lignes):
            texte.set_text(ligne)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        return True
