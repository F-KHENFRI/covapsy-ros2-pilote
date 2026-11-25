"""
Messages ROS 2 des capteurs proprioceptifs et repères tf2, communs au simulateur Python
et au driver Webots (mêmes topics, mêmes repères) :

  imu        sensor_msgs/Imu        repère <voiture>/base_link
  odom       nav_msgs/Odometry      <voiture>/odom -> <voiture>/base_link
  tf         tf2_msgs/TFMessage     <voiture>/odom -> <voiture>/base_link (dynamique)
  tf_static  tf2_msgs/TFMessage     <voiture>/base_link -> capteurs (lidar, ultrasons,
                                    caméra), QoS « transient local »
"""
import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

# QoS des publications tf2 (identiques à tf2_ros.TransformBroadcaster / Static...)
QOS_TF = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
QOS_TF_STATIC = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)


def quaternion(lacet=0.0, tangage=0.0):
    """(x, y, z, w) d'une rotation lacet (z) puis tangage (y)."""
    cy, sy = math.cos(lacet / 2), math.sin(lacet / 2)
    cp, sp = math.cos(tangage / 2), math.sin(tangage / 2)
    return (-sy * sp, cy * sp, sy * cp, cy * cp)


def _orienter(q, cap):
    q.x, q.y, q.z, q.w = quaternion(cap)


def messages(p, stamp, voiture):
    """(Imu, Odometry, TFMessage odom -> base_link) à partir d'une Proprioception."""
    base, odom = f'{voiture}/base_link', f'{voiture}/odom'
    imu = Imu()
    imu.header.stamp = stamp
    imu.header.frame_id = base
    _orienter(imu.orientation, p.cap)                 # cap intégré du gyroscope (dérive)
    imu.orientation_covariance[8] = 0.01
    imu.angular_velocity.z = p.gyro_z
    imu.angular_velocity_covariance[8] = p.p.gyro_bruit ** 2
    imu.linear_acceleration.x, imu.linear_acceleration.y = p.acc_x, p.acc_y
    imu.linear_acceleration.z = 9.81
    for i in (0, 4, 8):
        imu.linear_acceleration_covariance[i] = p.p.acc_bruit ** 2
    od = Odometry()
    od.header.stamp = stamp
    od.header.frame_id, od.child_frame_id = odom, base
    od.pose.pose.position.x, od.pose.pose.position.y = p.x, p.y
    _orienter(od.pose.pose.orientation, p.cap)
    od.twist.twist.linear.x = p.v_roue
    od.twist.twist.angular.z = p.gyro_z
    od.twist.covariance[0] = p.p.roue_bruit ** 2
    od.twist.covariance[35] = p.p.gyro_bruit ** 2
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id, t.child_frame_id = odom, base
    t.transform.translation.x, t.transform.translation.y = p.x, p.y
    _orienter(t.transform.rotation, p.cap)
    return imu, od, TFMessage(transforms=[t])


def statiques(stamp, voiture, capteurs):
    """TFMessage base_link -> capteurs ; capteurs = [(repère, x, y, z, lacet, tangage)]."""
    out = []
    for nom, x, y, z, lacet, tangage in capteurs:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id, t.child_frame_id = f'{voiture}/base_link', nom
        t.transform.translation.x, t.transform.translation.y = float(x), float(y)
        t.transform.translation.z = float(z)
        q = t.transform.rotation
        q.x, q.y, q.z, q.w = quaternion(lacet, tangage)
        out.append(t)
    return TFMessage(transforms=out)
