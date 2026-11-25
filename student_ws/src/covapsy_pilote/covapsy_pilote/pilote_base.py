"""
Pilote de BASE (copie figée du nœud pilote d'origine).  NE PAS MODIFIER.

Sert d'ADVERSAIRE dans « covapsy duel » (exécutable « adversaire », niveaux lent / moyen /
rapide réglés par paramètres dans launch/duel.launch.py). Votre code est dans pilote.py.
"""
import math
import time

from ackermann_msgs.msg import AckermannDriveStamped
from covapsy_interfaces.msg import RaceState, Ultrasons
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

from covapsy_pilote.outils_scan import distance_secteur, limiter, lisser, scan_vers_tableaux

BRAQUAGE_MAX = 0.35          # rad (≈ 20°), butée mécanique de la TT02
EMPATTEMENT = 0.257          # m

# race/state est publié en « transient local » : le dernier état est rejoué à l'abonnement
QOS_ETAT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Paramètres ROS 2 : nom -> (valeur par défaut, description)
PARAMETRES = {
    'vitesse': (1.0, 'm/s, vitesse CONSTANTE en course (AMÉLIORATION 1)'),
    'distance_securite': (1.0, 'm, anti-collision : ralentir si un obstacle est plus proche, '
                               'droit devant'),
    'k_direction': (1.0, 'gain : braquage = k_direction x angle visé'),
    'k_murs': (0.8, "gain du garde-fou : s'écarter du mur le plus proche"),
    'distance_murs': (0.8, "m, le garde-fou n'agit qu'en dessous de cette distance"),
    'champ_deg': (90.0, 'degrés, on ne regarde que devant : +/- champ'),
    'portee': (4.0, 'm, distances saturées au-delà'),
    'lissage': (15, 'points, lissage du scan (1 point par degré)'),
    'vitesse_recul': (0.5, 'm/s, vitesse de marche arrière'),
}


class Pilote(Node):
    """Pilote de base : viser la direction la plus dégagée, à vitesse constante."""

    def __init__(self, nom='pilote'):
        super().__init__(nom)
        # 1. PARAMÈTRES (lus à chaque scan : un « ros2 param set » agit immédiatement ;
        #    « ros2 param describe /pilote vitesse » affiche la description)
        for cle, (defaut, description) in PARAMETRES.items():
            self.declare_parameter(cle, defaut, ParameterDescriptor(description=description))

        # 2. ÉTAT INTERNE (mémoire entre deux appels)
        self.etat = RaceState.WAITING
        self.us = Ultrasons(avant=math.inf, arriere=math.inf, gauche=math.inf,
                            droite=math.inf)   # dernières distances ultrasons (m)
        self.cote_recul = None       # côté choisi pendant une marche arrière

        # 3. ABONNEMENTS (entrées)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(RaceState, 'race/state', self.on_etat, QOS_ETAT)
        self.create_subscription(Ultrasons, 'ultrasons', self.on_ultrasons,
                                 qos_profile_sensor_data)

        # 4. PUBLICATIONS (sorties)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, 'drive_raw', 10)
        self.pub_cible = self.create_publisher(Float32, 'pilote/angle_cible', 10)
        self.pub_marqueur = self.create_publisher(Marker, 'pilote/marqueur', 10)

        self.get_logger().info('pilote prêt : en attente de /scan et de /race/state')

    def p(self, cle):
        """Valeur courante d'un paramètre ROS 2."""
        return self.get_parameter(cle).value

    # ---------------------------------------------------------------------------------
    # CALLBACKS
    # ---------------------------------------------------------------------------------
    def on_etat(self, msg: RaceState):
        if msg.state != self.etat:
            self.get_logger().info(f'état de course : {msg.state} ({msg.reason})')
        self.etat = msg.state

    def on_ultrasons(self, msg: Ultrasons):
        """Les 4 ultrasons (avant, arriere, gauche, droite) en m ; inf = rien à moins de 2 m."""
        self.us = msg

    def on_scan(self, msg: LaserScan):
        """Appelé à chaque tour du lidar (~10 Hz) : décider puis publier une commande."""
        t0 = time.perf_counter()
        angles, distances = scan_vers_tableaux(msg)

        if self.etat == RaceState.RACING:
            self.cote_recul = None
            braquage, angle_cible = self.calculer_braquage(angles, distances)
            vitesse = self.calculer_vitesse(angles, distances, braquage)
            self.publier_debogage(msg, angle_cible, angles, distances)
        elif self.etat == RaceState.REVERSING:
            vitesse, braquage = self.se_degager(angles, distances)
        else:                                            # WAITING ou STOPPED
            self.cote_recul = None
            vitesse, braquage = 0.0, 0.0

        cmd = AckermannDriveStamped()
        cmd.header.stamp = msg.header.stamp
        cmd.drive.speed = float(vitesse)
        cmd.drive.steering_angle = float(braquage)
        self.pub_drive.publish(cmd)

        calcul_ms = (time.perf_counter() - t0) * 1e3
        self.get_logger().info(
            f'état {self.etat}  v {vitesse:+.2f} m/s  braquage {math.degrees(braquage):+5.1f}°'
            f'  ({calcul_ms:.1f} ms)', throttle_duration_sec=1.0)

    # ---------------------------------------------------------------------------------
    # DÉBOGAGE : rendre visible ce que le nœud a décidé
    # ---------------------------------------------------------------------------------
    def publier_debogage(self, scan, angle, angles, distances):
        """Direction visée : angle (Float32) et flèche rviz2 (Marker) dans le repère du lidar."""
        self.pub_cible.publish(Float32(data=float(angle)))
        i = int(np.argmin(np.abs(angles - angle)))           # rayon le plus proche de l'angle
        longueur = float(min(distances[i], self.p('portee')))  # = distance libre visée
        m = Marker()
        m.header = scan.header                               # même repère et même instant
        m.ns, m.id = 'pilote', 0
        m.type, m.action = Marker.ARROW, Marker.ADD
        m.points = [Point(), Point(x=longueur * math.cos(angle), y=longueur * math.sin(angle))]
        m.scale.x, m.scale.y, m.scale.z = 0.03, 0.08, 0.1    # tige, pointe (m)
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.8, 0.2, 1.0
        m.lifetime.nanosec = 500_000_000                     # disparaît si le nœud se tait
        self.pub_marqueur.publish(m)

    # ---------------------------------------------------------------------------------
    # DIRECTION
    # ---------------------------------------------------------------------------------
    def calculer_braquage(self, angles, distances):
        """Braquer vers la direction la plus dégagée, en s'écartant des murs proches.

        AMÉLIORATION 2 : est-ce la meilleure trajectoire ? Observez où passe la voiture
        dans les virages, près du mur intérieur et autour des obstacles.
        """
        angle = self.direction_plus_degagee(angles, distances)
        lim = self.p('distance_murs')
        g = min(distance_secteur(angles, distances, 15, 120), lim)     # mur à gauche
        d = min(distance_secteur(angles, distances, -120, -15), lim)   # mur à droite
        braquage = self.p('k_direction') * angle + self.p('k_murs') * (g - d)
        return limiter(braquage, -BRAQUAGE_MAX, BRAQUAGE_MAX), angle

    def direction_plus_degagee(self, angles, distances):
        """Angle (rad) de la direction la plus dégagée devant la voiture."""
        devant = np.abs(angles) <= math.radians(self.p('champ_deg'))
        if not np.any(devant):
            return 0.0
        a = angles[devant]
        d = lisser(np.minimum(distances[devant], self.p('portee')), int(self.p('lissage')))
        meilleurs = np.flatnonzero(d >= d.max() - 0.05)     # directions ~maximales
        return float(a[meilleurs[len(meilleurs) // 2]])

    # ---------------------------------------------------------------------------------
    # VITESSE
    # ---------------------------------------------------------------------------------
    def calculer_vitesse(self, angles, distances, braquage):
        """Vitesse CONSTANTE, la même en ligne droite et en virage.

        Seule exception, l'ANTI-COLLISION : si un obstacle (mur, voiture) est à moins de
        distance_securite droit devant (lidar ou ultrason avant), la vitesse baisse,
        jusqu'à 30 % de la vitesse au contact.

        AMÉLIORATION 1 : quelle vitesse constante maximale passe sans choc ?
        AMÉLIORATION 3 : à vous d'inventer une vitesse VARIABLE. Questions pour démarrer :
        que se passe-t-il en virage quand on va trop vite ? Comment savoir, avec le scan
        seul, qu'un virage arrive ? Quelle distance faut-il pour freiner ?
        (données physiques : README, §8)
        """
        devant = min(distance_secteur(angles, distances, -10, 10), self.us.avant)
        facteur = limiter(devant / self.p('distance_securite'), 0.3, 1.0)
        return self.p('vitesse') * facteur

    # ---------------------------------------------------------------------------------
    # MARCHE ARRIÈRE (race_manager a détecté un blocage : état REVERSING)
    # ---------------------------------------------------------------------------------
    def se_degager(self, angles, distances):
        """Reculer en orientant le nez vers le côté le plus libre.

        En marche arrière, roues à DROITE = le nez pivote vers la GAUCHE.
        AMÉLIORATION 4 : chaque marche arrière coûte ~2 s : mieux vaut l'éviter.
        """
        if self.cote_recul is None:                      # choisi une fois par manœuvre
            g = distance_secteur(angles, distances, 60, 120)
            d = distance_secteur(angles, distances, -120, -60)
            self.cote_recul = 1.0 if g >= d else -1.0
        arriere = min(distance_secteur(angles, distances, 160, -160), self.us.arriere)
        braquage = -self.cote_recul * BRAQUAGE_MAX
        if arriere < 0.15:
            return 0.0, braquage
        return -self.p('vitesse_recul'), braquage


def main(args=None, classe=Pilote):
    rclpy.init(args=args)
    noeud = classe()
    try:
        rclpy.spin(noeud)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:                                             # 2e Ctrl+C : contexte déjà fermé
            noeud.destroy_node()
            rclpy.try_shutdown()
        except (Exception, KeyboardInterrupt):           # noqa: B902
            pass


def main_adversaire(args=None):
    main(args, classe=lambda: Pilote('adversaire'))
