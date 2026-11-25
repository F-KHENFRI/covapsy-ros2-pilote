"""
TP3 – Machine à états de course, services race/start et race/stop.

WAITING --start--> RACING --bloqué & arrière libre--> REVERSING --avant dégagé|t_max--> RACING
(tout état) --stop--> STOPPED --start--> RACING
(tout état) --reset--> WAITING   (nouvelle course, appelé par la direction de course)
"""
from covapsy_autonomy.common import declare, Params, RACE_STATE_QOS, run
from covapsy_interfaces.msg import RaceState, ScanSummary, Ultrasons
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

NAMES = {RaceState.WAITING: 'WAITING', RaceState.RACING: 'RACING',
         RaceState.REVERSING: 'REVERSING', RaceState.STOPPED: 'STOPPED'}


class RaceManager(Node):

    def __init__(self) -> None:
        super().__init__('race_manager')
        declare(self, 'd_block', 0.25, 'Bloqué si front < d_block (m)', 0.05, 2.0)
        declare(self, 't_block', 1.0, 'Durée de blocage avant marche arrière (s)', 0.1, 5.0)
        declare(self, 'd_rear_clear', 0.30, 'Marche arrière permise si rear > (m)', 0.05, 2.0)
        declare(self, 'd_clear', 0.6, 'Fin de marche arrière si front > (m)', 0.1, 3.0)
        declare(self, 't_reverse_max', 2.0, 'Durée maximale de marche arrière (s)', 0.2, 5.0)
        declare(self, 'use_ultrasound', True,
                'Combine lidar et ultrasons avant / arrière (topic ultrasons, min des deux)')
        declare(self, 'autostart_delay', -1.0,
                'Si >= 0 : race/start automatique après ce délai (s), pour les essais')
        self.p = Params(self)

        self.state = RaceState.WAITING
        self.reason = 'initialisation'
        self.summary: ScanSummary | None = None
        self.us = {'front': float('inf'), 'rear': float('inf')}
        self.wrong_way = False
        self.blocked_since = None
        self.reverse_since = None
        self.t0 = self.get_clock().now()

        self.pub = self.create_publisher(RaceState, 'race/state', RACE_STATE_QOS)
        self.create_subscription(ScanSummary, 'perception/scan_summary', self.on_summary, 10)
        self.create_subscription(Ultrasons, 'ultrasons', self.on_us, qos_profile_sensor_data)
        self.create_subscription(Bool, 'perception/wrong_way', self.on_wrong_way, 10)
        self.create_service(Trigger, 'race/start', self.on_start)
        self.create_service(Trigger, 'race/stop', self.on_stop)
        self.create_service(Trigger, 'race/reset', self.on_reset)
        self.create_timer(0.1, self.on_timer)
        self.publish()

    # --- Entrées ---------------------------------------------------------------
    def on_summary(self, msg: ScanSummary) -> None:
        self.summary = msg

    def on_us(self, msg: Ultrasons) -> None:
        # les 4 ultrasons dans un message ; inf = rien à portée
        self.us['front'], self.us['rear'] = msg.avant, msg.arriere

    def on_wrong_way(self, msg: Bool) -> None:
        if msg.data and not self.wrong_way:
            self.get_logger().warn('Contresens détecté par la caméra')
            self.reason = 'alerte contresens'
            self.publish()
        self.wrong_way = msg.data

    def on_start(self, request, response):
        if self.state in (RaceState.WAITING, RaceState.STOPPED):
            self.transition(RaceState.RACING, 'race/start')
            response.success, response.message = True, 'départ'
        else:
            response.success, response.message = False, f'déjà en {NAMES[self.state]}'
        return response

    def on_stop(self, request, response):
        prev = NAMES[self.state]
        self.transition(RaceState.STOPPED, 'race/stop')
        response.success, response.message = True, f'arrêt (était {prev})'
        return response

    def on_reset(self, request, response):
        prev = NAMES[self.state]
        self.transition(RaceState.WAITING, 'race/reset')
        response.success, response.message = True, f'remise sur la grille (était {prev})'
        return response

    # --- Machine à états -------------------------------------------------------
    def transition(self, state, reason) -> None:
        if state != self.state:
            self.get_logger().info(f'{NAMES[self.state]} -> {NAMES[state]} ({reason})')
        self.state, self.reason = state, reason
        self.blocked_since = None
        self.reverse_since = self.get_clock().now() if state == RaceState.REVERSING else None
        self.publish()

    def distance(self, side) -> float:
        d = getattr(self.summary, side)
        if self.p.use_ultrasound:
            d = min(d, self.us[side])
        return d

    def on_timer(self) -> None:
        now = self.get_clock().now()
        if (self.p.autostart_delay >= 0.0 and self.state == RaceState.WAITING
                and now - self.t0 > Duration(seconds=self.p.autostart_delay)):
            self.transition(RaceState.RACING, 'autostart')

        if self.summary is not None and self.state == RaceState.RACING:
            front = self.distance('front')
            if front < self.p.d_block:
                if self.blocked_since is None:
                    self.blocked_since = now
                elif now - self.blocked_since > Duration(seconds=self.p.t_block):
                    rear = self.distance('rear')
                    if rear > self.p.d_rear_clear:
                        self.transition(RaceState.REVERSING, f'bloqué (front {front:.2f} m)')
                    else:
                        self.get_logger().warn(f'bloqué mais arrière occupé ({rear:.2f} m)',
                                               throttle_duration_sec=1.0)
            else:
                self.blocked_since = None

        elif self.summary is not None and self.state == RaceState.REVERSING:
            front = self.distance('front')
            if front > self.p.d_clear:
                self.transition(RaceState.RACING, f'dégagé (front {front:.2f} m)')
            elif now - self.reverse_since > Duration(seconds=self.p.t_reverse_max):
                self.transition(RaceState.RACING, 'durée max de marche arrière')
            elif self.distance('rear') < 0.5 * self.p.d_rear_clear:
                self.transition(RaceState.RACING, 'obstacle derrière')
        self.publish()

    def publish(self) -> None:
        msg = RaceState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self.state
        msg.reason = self.reason + (' [contresens]' if self.wrong_way else '')
        self.pub.publish(msg)


def main(args=None) -> None:
    run(RaceManager, args)
