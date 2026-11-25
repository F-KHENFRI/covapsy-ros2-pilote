"""
Nœud « chronometre » : chronométrage d'une course dans le simulateur Python.

Il lit la vérité terrain du simulateur (/simulateur/verite_terrain), compte les tours au
franchissement de la ligne d'arrivée, retire de la piste les voitures arrivées
(/simulateur/consigne), puis affiche le classement, l'écrit en JSON (paramètre
``resultat``) et S'ARRÊTE, ce qui termine le launch (chrono.launch.py, duel.launch.py).

Paramètres : circuit, voitures (noms dans le simulateur), noms (affichés), tours,
duree_max (s simulées), resultat (fichier JSON, facultatif).
Temps total = du départ de la simulation (t = 0) à la fin du dernier tour.
"""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from covapsy_simple_sim.carte import charger_circuit, franchit


class Chronometre(Node):

    def __init__(self):
        super().__init__('chronometre')
        self.declare_parameter('circuit', 'ovale')
        self.declare_parameter('voitures', ['car1'])
        self.declare_parameter('noms', [''])
        self.declare_parameter('tours', 3)
        self.declare_parameter('duree_max', 180.0)
        self.declare_parameter('resultat', '')
        g = lambda k: self.get_parameter(k).value                  # noqa: E731
        self.ligne = charger_circuit(g('circuit'), obstacles=False).ligne_arrivee
        self.circuit, self.tours, self.duree_max = g('circuit'), g('tours'), g('duree_max')
        noms = [n for n in g('noms') if n]
        self.voitures = {v: {'nom': noms[i] if i < len(noms) else v, 'pos': None,
                             't_tour': None, 'temps': [], 'total': None, 'chocs': 0,
                             'distance': 0.0}
                         for i, v in enumerate(g('voitures'))}
        self.sens = None
        self.t = 0.0
        self.fini = False
        self.pub = self.create_publisher(String, '/simulateur/consigne', 10)
        self.create_subscription(String, '/simulateur/verite_terrain', self.on_verite, 10)
        self.get_logger().info(f'chrono : {self.tours} tours sur « {self.circuit} », '
                               f'abandon après {self.duree_max:.0f} s')

    def on_verite(self, msg):
        d = json.loads(msg.data)
        self.t = float(d['t'])
        for v, c in self.voitures.items():
            vt = d['voitures'].get(v)
            if vt is None or c['total'] is not None:
                continue
            p = (vt['x'], vt['y'])
            c['chocs'], c['distance'] = vt['collisions'], vt['distance']
            if c['pos'] is not None:
                f = franchit(c['pos'], p, self.ligne)
                self.sens = self.sens or (f if f else None)     # 1er passage = sens course
                if f and f == self.sens:
                    self.passage(v, c)
            c['pos'] = p
        if all(c['total'] is not None for c in self.voitures.values()) \
                or self.t > self.duree_max:
            self.terminer()

    def passage(self, v, c):
        if c['t_tour'] is not None:
            c['temps'].append(self.t - c['t_tour'])
            self.get_logger().info(f"{c['nom']} : tour {len(c['temps'])}/{self.tours} "
                                   f"en {c['temps'][-1]:.2f} s")
            if len(c['temps']) >= self.tours:
                c['total'] = self.t
                self.get_logger().info(f"*** {c['nom']} ARRIVÉE en {self.t:.2f} s ***")
                self.pub.publish(String(data=json.dumps({'action': 'retirer', 'voiture': v})))
        c['t_tour'] = self.t

    def classement(self):
        return sorted(self.voitures.items(), key=lambda kv: (
            kv[1]['total'] is None, kv[1]['total'] or 0.0, -len(kv[1]['temps']),
            -kv[1]['distance']))

    def terminer(self):
        if self.fini:
            return
        self.fini = True
        lignes = [f'Résultat « {self.circuit} » ({self.tours} tours, t = {self.t:.1f} s)',
                  f'{"#":>2} {"pilote":16s} {"tours":>5s} {"total":>7s} {"meilleur":>8s} '
                  f'{"chocs":>5s}']
        resultat = []
        for i, (v, c) in enumerate(self.classement(), 1):
            meilleur = min(c['temps']) if c['temps'] else None
            lignes.append(f"{i:2d} {c['nom'][:16]:16s} {len(c['temps']):5d} "
                          + (f"{c['total']:7.2f}" if c['total'] is not None else '      -')
                          + (f" {meilleur:8.2f}" if meilleur else '        -')
                          + f" {c['chocs']:5d}")
            resultat.append({'voiture': v, 'nom': c['nom'], 'position': i,
                             'tours': len(c['temps']), 'total': c['total'],
                             'meilleur': meilleur, 'temps_tours': c['temps'],
                             'chocs': c['chocs']})
        self.get_logger().info('\n' + '\n'.join(lignes))
        fichier = self.get_parameter('resultat').value
        if fichier:
            with open(fichier, 'w') as f:
                json.dump({'circuit': self.circuit, 'tours': self.tours,
                           'duree_max': self.duree_max, 'classement': resultat}, f)
        raise SystemExit                          # fin du nœud -> fin du launch


def main(args=None):
    rclpy.init(args=args)
    noeud = Chronometre()
    try:
        rclpy.spin(noeud)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        try:
            noeud.destroy_node()
            rclpy.try_shutdown()
        except (Exception, KeyboardInterrupt):  # noqa: B902
            pass
