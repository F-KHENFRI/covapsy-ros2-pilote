"""
TP3 – Supervision indépendante : dernière barrière avant l'actionneur.

Entrées : drive_raw, perception/scan_summary, race/state (+ ultrasons : avant, arrière).
Sortie  : drive, publiée par un timer à 50 Hz (pas cadencé par les entrées).
"""
from ackermann_msgs.msg import AckermannDriveStamped
from covapsy_autonomy.common import declare, Params, RACE_STATE_QOS, run
from covapsy_interfaces.msg import RaceState, ScanSummary, Ultrasons
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class SafetyWatchdog(Node):

    def __init__(self) -> None:
        super().__init__('safety_watchdog')
        declare(self, 'cmd_timeout', 0.3, 'Commande nulle si drive_raw plus vieux (s)', 0.05, 2.0)
        declare(self, 'scan_timeout', 0.3, 'Commande nulle si scan plus vieux (s)', 0.05, 2.0)
        declare(self, 'd_emergency', 0.15, "Freinage d'urgence si front < (m)", 0.0, 1.0)
        declare(self, 'd_emergency_rear', 0.10, 'Arrêt de la marche arrière si rear < (m)',
                0.0, 1.0)
        declare(self, 'v_abs', 3.0, 'Vitesse absolue maximale (m/s)', 0.0, 5.0)
        declare(self, 'max_steering_angle', 0.30, 'Braquage maximal (rad)', 0.0, 0.6)
        declare(self, 'use_ultrasound', True,
                'Redondance : ultrasons avant et arrière (topic ultrasons) indépendants du lidar')
        self.p = Params(self)

        self.cmd: AckermannDriveStamped | None = None
        self.cmd_time = None
        self.summary: ScanSummary | None = None
        self.summary_time = None
        self.state = RaceState.WAITING
        self.us = {'front': float('inf'), 'rear': float('inf')}

        self.create_subscription(AckermannDriveStamped, 'drive_raw', self.on_cmd, 10)
        self.create_subscription(ScanSummary, 'perception/scan_summary', self.on_summary, 10)
        self.create_subscription(RaceState, 'race/state', self.on_state, RACE_STATE_QOS)
        self.create_subscription(Ultrasons, 'ultrasons', self.on_us, qos_profile_sensor_data)
        self.pub = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        self.create_timer(1.0 / 50.0, self.on_timer)

    def on_cmd(self, msg) -> None:
        self.cmd, self.cmd_time = msg, self.get_clock().now()

    def on_summary(self, msg) -> None:
        self.summary, self.summary_time = msg, self.get_clock().now()

    def on_state(self, msg) -> None:
        self.state = msg.state

    def on_us(self, msg: Ultrasons) -> None:
        self.us['front'], self.us['rear'] = msg.avant, msg.arriere   # inf = rien à portée

    def intervene(self, rule) -> None:
        self.get_logger().warn(f'intervention : {rule}', throttle_duration_sec=1.0)

    def stale(self, t, timeout, now) -> bool:
        return t is None or now - t > Duration(seconds=timeout)

    def on_timer(self) -> None:
        p = self.p
        now = self.get_clock().now()
        out = AckermannDriveStamped()
        out.header.stamp = now.to_msg()

        if self.state not in (RaceState.RACING, RaceState.REVERSING):
            pass                                         # WAITING / STOPPED : commande nulle
        elif self.stale(self.cmd_time, p.cmd_timeout, now):
            self.intervene('drive_raw absent')
        elif self.stale(self.summary_time, p.scan_timeout, now):
            self.intervene("scan absent : conduite à l'aveugle")
        else:
            v = self.cmd.drive.speed
            delta = self.cmd.drive.steering_angle
            # tolérance : AckermannDrive est en float32 (0.30 devient 0.3000000119)
            if abs(v) > p.v_abs + 1e-4:
                self.intervene('saturation vitesse')
                v = max(-p.v_abs, min(p.v_abs, v))
            if abs(delta) > p.max_steering_angle + 1e-4:
                self.intervene('saturation braquage')
                delta = max(-p.max_steering_angle, min(p.max_steering_angle, delta))
            front = self.summary.front
            rear = self.summary.rear
            if p.use_ultrasound:
                front = min(front, self.us['front'])
                rear = min(rear, self.us['rear'])
            if v > 0.0 and front < p.d_emergency:
                self.intervene(f"freinage d'urgence (front {front:.2f} m)")
                v = 0.0
            if v < 0.0 and rear < p.d_emergency_rear:
                self.intervene(f'arrêt marche arrière (rear {rear:.2f} m)')
                v = 0.0
            out.drive.speed, out.drive.steering_angle = float(v), float(delta)
        self.pub.publish(out)


def main(args=None) -> None:
    run(SafetyWatchdog, args)
