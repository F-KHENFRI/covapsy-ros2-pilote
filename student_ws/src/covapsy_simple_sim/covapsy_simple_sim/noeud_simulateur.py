"""
Simulateur Python léger : MÊMES interfaces ROS 2 que la voiture Webots.

Par voiture (namespace ``carN``, ou aucun namespace si ``sans_namespace``) :
  publie   scan (sensor_msgs/LaserScan, ~11 Hz, 360 points, sens horaire comme Webots)
           ultrasons (covapsy_interfaces/Ultrasons, 20 Hz) : les 4 ultrasons dans UN message
           imu (sensor_msgs/Imu, 50 Hz) : gyroscope, accéléromètre, cap intégré (dérive)
           odom (nav_msgs/Odometry, 50 Hz) : odométrie à l'estime (roue + gyroscope), à
           partir de (0, 0, 0) sur la grille ; elle dérive : ce n'est PAS la vérité terrain
           tf (odom -> base_link), tf_static (base_link -> laser, us_*, camera)
  service  perception/sens_circuit (covapsy_interfaces/SensCircuit) : la voiture roule-t-elle
           dans le bon sens ? D'après la caméra simulée (bordures vertes / rouges vues à
           gauche et à droite, moyennées sur ~1 s), comme wall_color_detector sous Webots
  souscrit drive (ackermann_msgs/AckermannDriveStamped)  <- safety_watchdog
           pilote/angle_cible (std_msgs/Float32, si affichage) : flèche « direction visée »
Global :
  publie   /clock (temps simulé)
Réservé à l'enseignant (jamais transmis aux équipes par la passerelle) :
  publie   /simulateur/verite_terrain (std_msgs/String, JSON, 10 Hz) : positions réelles
           /simulateur/info (std_msgs/String, JSON, transient local) : circuit, voitures
  souscrit /simulateur/consigne (std_msgs/String, JSON) :
           {"action": "reinitialiser"} | {"action": "retirer", "voiture": "car2"}
           {"action": "pause"} | {"action": "reprendre"} | {"action": "immobiliser"}

Paramètres principaux : circuit, obstacles, voitures, sans_namespace, facteur_temps_reel
(1 = temps réel, 0 = aussi vite que possible), affichage (fenêtre matplotlib),
lidar_bruit, lidar_perte, graine, couleur_droite (vert | rouge : couleur du mur de droite
dans le sens de course ; vide = valeur du circuit, sinon vert).
"""
import json
import math
import os
import time

from ackermann_msgs.msg import AckermannDriveStamped
from covapsy_interfaces.msg import Ultrasons
from covapsy_interfaces.srv import SensCircuit
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float32, String
from tf2_msgs.msg import TFMessage

from covapsy_simple_sim.carte import charger_circuit, franchit
from covapsy_simple_sim import proprio_ros
from covapsy_simple_sim.monde import Monde, ParametresCapteurs
from covapsy_simple_sim.odometrie import Proprioception
from covapsy_simple_sim.sens import EstimateurSens

LATCH = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Simulateur(Node):

    def __init__(self):
        super().__init__('simulateur')
        dp = self.declare_parameter
        dp('circuit', 'ovale')
        dp('obstacles', True)
        dp('voitures', ['car1'])
        dp('sans_namespace', False)
        dp('facteur_temps_reel', 1.0)
        dp('pas', 0.01)
        dp('frequence_lidar', 11.0)
        dp('frequence_us', 20.0)
        dp('frequence_imu', 50.0)
        dp('lidar_bruit', 0.005)
        dp('lidar_perte', 0.0)
        dp('graine', 0)
        dp('affichage', False)
        dp('couleur_droite', '')
        g = lambda k: self.get_parameter(k).value   # noqa: E731

        self.circuit = charger_circuit(g('circuit'), obstacles=g('obstacles'),
                                       couleur_droite=g('couleur_droite') or None)
        self.noms = list(g('voitures'))
        capteurs = ParametresCapteurs(lidar_bruit=g('lidar_bruit'), lidar_perte=g('lidar_perte'))
        self.monde = Monde(self.circuit, self.noms, capteurs=capteurs, graine=g('graine'))
        # IMU + odométrie : générateur propre à chaque voiture (n'altère pas le bruit lidar)
        self.proprio = {n: Proprioception(np.random.default_rng(g('graine') + 1000 + i))
                        for i, n in enumerate(self.noms)}
        self.periode_imu = 1.0 / g('frequence_imu')
        self.t_imu = -1.0
        self.dt = g('pas')
        self.facteur = g('facteur_temps_reel')
        self.periode_lidar, self.periode_us = 1.0 / g('frequence_lidar'), 1.0 / g('frequence_us')
        self.t_lidar = self.t_us = self.t_verite = -1.0
        self.pause = False
        self.sens = 'normal'                # sens de course (chronométrage local)
        self.tours = {n: {'n': -1, 't': None, 'meilleur': None, 'dernier': None} for n in self.noms}

        self.pub_clock = self.create_publisher(Clock, '/clock', 10)
        self.pubs = {}
        self.sens_vus = {n: EstimateurSens() for n in self.noms}   # couleurs vues, moyennées
        self.cibles = {}                    # voiture -> (angle visé par le pilote, instant)
        sans_ns = g('sans_namespace') and len(self.noms) == 1
        for nom in self.noms:
            ns = '' if sans_ns else f'/{nom}'
            self.pubs[nom] = {
                'scan': self.create_publisher(LaserScan, f'{ns}/scan', 10),
                'ultrasons': self.create_publisher(Ultrasons, f'{ns}/ultrasons', 10),
                'imu': self.create_publisher(Imu, f'{ns}/imu', 10),
                'odom': self.create_publisher(Odometry, f'{ns}/odom', 10),
                # tf2 : dans le namespace de la voiture (/carN/tf) pour que chaque équipe
                # ne reçoive que SA voiture (passerelle : /carN/tf -> /tf de l'équipe)
                'tf': self.create_publisher(TFMessage, f'{ns}/tf', proprio_ros.QOS_TF),
                'tf_static': self.create_publisher(TFMessage, f'{ns}/tf_static',
                                                   proprio_ros.QOS_TF_STATIC),
                }
            self.create_subscription(AckermannDriveStamped, f'{ns}/drive',
                                     lambda m, n=nom: self.on_drive(n, m), 10)
            # sens de circulation d'après la caméra simulée : un service par voiture
            self.create_service(SensCircuit, f'{ns}/perception/sens_circuit',
                                lambda q, r, n=nom: self.on_sens(n, q, r))
            if g('affichage'):
                # débogage du pilote : direction visée, dessinée dans la fenêtre (flèche)
                self.create_subscription(
                    Float32, f'{ns}/pilote/angle_cible',
                    lambda m, n=nom: self.cibles.__setitem__(n, (m.data, time.monotonic())), 10)
        self.pub_verite = self.create_publisher(String, '/simulateur/verite_terrain', 10)
        self.pub_info = self.create_publisher(String, '/simulateur/info', LATCH)
        self.create_subscription(String, '/simulateur/consigne', self.on_consigne, 10)
        self.publier_info()
        for nom in self.noms:
            self.publier_tf_statiques(nom)

        if self.facteur > 0:
            self.create_timer(self.dt / self.facteur, lambda: self.avancer(1))
        else:
            self.create_timer(0.002, lambda: self.avancer(10))     # aussi vite que possible
        self.vue = None
        if g('affichage'):
            self.ouvrir_vue()
        self.get_logger().info(
            f'Simulateur prêt : circuit « {self.circuit.nom} » ({self.circuit.longueur:.1f} m), '
            f'voitures {self.noms}' + (' (sans namespace)' if sans_ns else '')
            + f', temps réel x{self.facteur if self.facteur > 0 else "max"}'
            + f', mur de droite {self.circuit.couleur_droite}')

    # --- Entrées -------------------------------------------------------------------------
    def on_drive(self, nom, msg):
        self.monde.commander(nom, msg.drive.speed, msg.drive.steering_angle)

    def on_consigne(self, msg):
        try:
            c = json.loads(msg.data)
            action = c.get('action')
            if action == 'reinitialiser':
                # facultatif : poses {voiture: [x, y, yaw]} et sens (normal | inverse),
                # choisis par la direction de course (rotation de grille, sens inverse)
                self.sens = c.get('sens', self.sens)
                self.monde.reinitialiser(c.get('poses'))
                for e in self.sens_vus.values():
                    e.reinitialiser()
                self.publier_info()
                self.tours = {n: {'n': -1, 't': None, 'meilleur': None, 'dernier': None}
                              for n in self.noms}
                self.t_lidar = self.t_us = self.t_verite = self.t_imu = -1.0
                for p in self.proprio.values():
                    p.reinitialiser()
                self.get_logger().info(f'réinitialisation : voitures sur la grille (sens '
                                       f'{self.sens})')
            elif action == 'retirer':
                self.monde.retirer(c['voiture'])
                self.get_logger().warning(f"{c['voiture']} retirée de la piste")
            elif action == 'immobiliser':                  # arrêt d'urgence
                for v in self.monde.voitures.values():
                    v.v = v.consigne_v = 0.0
                self.get_logger().warning("arrêt d'urgence : toutes les voitures immobilisées")
            elif action in ('pause', 'reprendre'):
                self.pause = action == 'pause'
                self.get_logger().info(f'simulation {"en pause" if self.pause else "reprise"}')
            else:
                raise ValueError(f'action inconnue : {action}')
        except Exception as e:                                  # noqa: B902
            self.get_logger().error(f'consigne invalide {msg.data!r} : {e}')

    # --- Boucle ----------------------------------------------------------------------------
    def avancer(self, n):
        if self.pause:
            return
        for _ in range(n):
            avant = {k: (v.x, v.y) for k, v in self.monde.voitures.items()}
            self.monde.pas(self.dt)
            for nom, v in self.monde.voitures.items():
                if not v.retiree:
                    self.proprio[nom].maj(v.v, v.yaw, self.dt)
            self.compter_tours(avant)
            t = self.monde.t
            self.pub_clock.publish(Clock(clock=self.stamp(t)))
            if t - self.t_lidar >= self.periode_lidar - 1e-9:
                self.t_lidar = t
                for nom in self.noms:
                    self.publier_scan(nom, t)
            if t - self.t_imu >= self.periode_imu - 1e-9:
                self.t_imu = t
                for nom in self.noms:
                    self.publier_proprio(nom, t)
            if t - self.t_us >= self.periode_us - 1e-9:
                self.t_us = t
                for nom in self.noms:
                    self.publier_us(nom, t)
            if t - self.t_verite >= 0.1 - 1e-9:
                self.t_verite = t
                self.pub_verite.publish(String(data=json.dumps(self.monde.verite_terrain())))
        if self.vue is not None:
            self.maj_vue()

    def compter_tours(self, avant):
        """Chronométrage local (affichage pédagogique ; l'officiel est fait par l'arbitre)."""
        for nom, v in self.monde.voitures.items():
            attendu = 1 if self.sens == 'normal' else -1
            if franchit(avant[nom], (v.x, v.y), self.circuit.ligne_arrivee) != attendu:
                continue
            k = self.tours[nom]
            k['n'] += 1
            if k['t'] is not None:
                k['dernier'] = self.monde.t - k['t']
                k['meilleur'] = min(k['meilleur'] or math.inf, k['dernier'])
                self.get_logger().info(f'{nom} : tour {k["n"]} en {k["dernier"]:.2f} s '
                                       f'(meilleur {k["meilleur"]:.2f} s)')
            k['t'] = self.monde.t

    @staticmethod
    def stamp(t):
        from builtin_interfaces.msg import Time
        sec = int(t)
        return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)

    def publier_scan(self, nom, t):
        a, d = self.monde.scan(nom)
        n = a.size
        # Webots : angle_min = +pi, angle_increment < 0 (sens horaire)
        idx = (n - np.arange(n)) % n
        msg = LaserScan()
        msg.header.stamp = self.stamp(t)
        msg.header.frame_id = f'{nom}/laser'
        msg.angle_min, msg.angle_max = math.pi, -math.pi + 2 * math.pi / n
        msg.angle_increment = -2 * math.pi / n
        msg.scan_time = self.periode_lidar
        msg.range_min, msg.range_max = self.monde.pc.lidar_portee_min, self.monde.pc.lidar_portee
        msg.ranges = d[idx].astype(np.float32).tolist()
        self.pubs[nom]['scan'].publish(msg)
        # caméra simulée (même cadence que le lidar) : alimente le service sens_circuit
        self.sens_vus[nom].maj(self.monde.couleurs_murs(nom), t)

    def publier_us(self, nom, t):
        mesures = self.monde.ultrasons(nom)          # {avant, arriere, gauche, droite} (inf)
        pc = self.monde.pc
        tous = Ultrasons(avant=mesures['avant'], arriere=mesures['arriere'],
                         gauche=mesures['gauche'], droite=mesures['droite'],
                         portee_min=pc.us_portee_min, portee_max=pc.us_portee)
        tous.header.stamp = self.stamp(t)
        tous.header.frame_id = f'{nom}/base_link'
        self.pubs[nom]['ultrasons'].publish(tous)

    def publier_proprio(self, nom, t):
        """IMU, odométrie et tf (odom -> base_link) MESURÉES (bruit, biais, dérive)."""
        imu, od, tf = proprio_ros.messages(self.proprio[nom], self.stamp(t), nom)
        self.pubs[nom]['imu'].publish(imu)
        self.pubs[nom]['odom'].publish(od)
        self.pubs[nom]['tf'].publish(tf)

    def publier_tf_statiques(self, nom):
        """Position des capteurs sur la voiture : base_link -> laser, us_*, camera."""
        pv = self.monde.pv
        hl, hw = pv.longueur / 2, pv.largeur / 2
        capteurs = [(f'{nom}/laser', pv.lidar_x, 0.0, 0.08, 0.0, 0.0),
                    (f'{nom}/us_front', hl, 0.0, 0.05, 0.0, 0.0),
                    (f'{nom}/us_rear', -hl, 0.0, 0.05, math.pi, 0.0),
                    (f'{nom}/us_left', 0.0, hw, 0.05, math.pi / 2, 0.0),
                    (f'{nom}/us_right', 0.0, -hw, 0.05, -math.pi / 2, 0.0),
                    (f'{nom}/camera', hl - 0.04, 0.0, 0.11, 0.0, 0.30)]
        self.pubs[nom]['tf_static'].publish(
            proprio_ros.statiques(self.stamp(self.monde.t), nom, capteurs))

    def on_sens(self, nom, requete, reponse):
        """Service perception/sens_circuit de la voiture ``nom``."""
        reponse.success, reponse.sens, reponse.score, reponse.message = \
            self.sens_vus[nom].repondre(requete.couleur_droite, requete.couleur_gauche,
                                        self.monde.t)
        return reponse

    def publier_info(self):
        c = self.circuit
        self.pub_info.publish(String(data=json.dumps({
            'circuit': c.nom, 'longueur': c.longueur, 'voitures': self.noms,
            'simulateur': 'python', 'sens': getattr(self, 'sens', 'normal')})))

    # --- Visualisation (facultative) ----------------------------------------------------
    def ouvrir_vue(self):
        if not os.environ.get('DISPLAY'):
            self.get_logger().warning('affichage demandé mais DISPLAY absent : fenêtre désactivée')
            return
        from covapsy_simple_sim.vue import Vue
        self.vue = Vue(self.circuit, self.monde, self.tours, cibles=self.cibles)

    def maj_vue(self):
        if not self.vue.maj(self.monde.t):
            self.vue = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Simulateur()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:                      # Ctrl+C : le contexte peut déjà être fermé
                node.destroy_node()
            except (Exception, KeyboardInterrupt):  # noqa: B902  (2e Ctrl+C)
                pass
        rclpy.try_shutdown()
