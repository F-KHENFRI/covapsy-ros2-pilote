"""Outils partagés par les nœuds : paramètres avec descripteurs, QoS, boucle main."""
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# race/state : un nœud démarré plus tard reçoit immédiatement l'état courant
RACE_STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)


def declare(node, name, default, description, lo=None, hi=None, step=0.0):
    """Déclare un paramètre avec description et bornes (visibles dans rqt_reconfigure)."""
    desc = ParameterDescriptor(description=description)
    if lo is not None and hi is not None:
        if isinstance(default, bool):
            pass
        elif isinstance(default, int):
            desc.integer_range = [IntegerRange(from_value=int(lo), to_value=int(hi), step=1)]
        elif isinstance(default, float):
            desc.floating_point_range = [FloatingPointRange(from_value=float(lo),
                                                            to_value=float(hi), step=step)]
    return node.declare_parameter(name, default, desc)


class Params:
    """Accès p.nom à la valeur courante d'un paramètre (réglage dynamique pris en compte)."""

    def __init__(self, node):
        object.__setattr__(self, '_node', node)

    def __getattr__(self, name):
        return self._node.get_parameter(name).value


def run(node_cls, args=None):
    """Boucle principale standard d'un nœud."""
    rclpy.init(args=args)
    node = node_cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:                      # Ctrl+C : le contexte peut déjà être fermé
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # noqa: B902  (2e Ctrl+C)
            pass
        try:
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass
