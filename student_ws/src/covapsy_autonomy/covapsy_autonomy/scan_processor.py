"""TP2/TP3 – Perception lidar : distances par secteur, marqueurs, ScanSummary."""
import math
import time

from covapsy_autonomy.common import declare, Params, run
from covapsy_autonomy.scan_utils import scan_to_arrays, sector_min, to_points
from covapsy_interfaces.msg import ScanSummary
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

D = math.radians


class ScanProcessor(Node):

    def __init__(self) -> None:
        super().__init__('scan_processor')
        for name, lo, hi in [('front', -15, 15), ('front_left', 15, 60),
                             ('front_right', -60, -15), ('left', 60, 120),
                             ('right', -120, -60), ('rear', 160, -160)]:
            declare(self, f'{name}_min', D(lo), f'Secteur {name} : angle min (rad)', -3.15, 3.15)
            declare(self, f'{name}_max', D(hi), f'Secteur {name} : angle max (rad)', -3.15, 3.15)
        declare(self, 'danger_distance', 0.5, 'Points affichés en zone de danger (m)', 0.0, 5.0)
        self.p = Params(self)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.pub_summary = self.create_publisher(ScanSummary, 'perception/scan_summary', 10)
        self.pub_markers = self.create_publisher(MarkerArray, 'perception/markers', 10)

    def sector(self, theta, r, name):
        return sector_min(theta, r, getattr(self.p, f'{name}_min'), getattr(self.p, f'{name}_max'))

    def on_scan(self, msg: LaserScan) -> None:
        t0 = time.perf_counter()
        theta, r = scan_to_arrays(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max)

        s = ScanSummary()
        s.header = msg.header
        for name in ('front', 'front_left', 'front_right', 'left', 'right', 'rear'):
            setattr(s, name, self.sector(theta, r, name))
        self.pub_summary.publish(s)
        self.pub_markers.publish(self.markers(msg, theta, r, s))

        dt_ms = (time.perf_counter() - t0) * 1e3
        self.get_logger().info(f'traitement scan : {dt_ms:.2f} ms ({r.size} points)',
                               throttle_duration_sec=1.0)

    def markers(self, msg, theta, r, s):
        arr = MarkerArray()
        for i, (name, ang) in enumerate([('front', 0.0), ('left', math.pi / 2),
                                         ('right', -math.pi / 2)]):
            m = Marker()
            m.header = msg.header
            m.ns, m.id = 'distances', i
            m.type, m.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            m.pose.position.x = 0.6 * math.cos(ang)
            m.pose.position.y = 0.6 * math.sin(ang)
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            m.scale.z = 0.12
            m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            m.text = f'{name} {getattr(s, name):.2f} m'
            arr.markers.append(m)

        danger = Marker()
        danger.header = msg.header
        danger.ns, danger.id = 'danger', 0
        danger.type, danger.action = Marker.POINTS, Marker.ADD
        danger.pose.orientation.w = 1.0
        danger.scale.x = danger.scale.y = 0.04
        danger.color = ColorRGBA(r=1.0, g=0.2, b=0.0, a=1.0)
        close = (r > 0.0) & (r < self.p.danger_distance)
        for x, y in to_points(theta[close], r[close]):
            danger.points.append(Point(x=float(x), y=float(y), z=0.0))
        arr.markers.append(danger)
        return arr


def main(args=None) -> None:
    run(ScanProcessor, args)
