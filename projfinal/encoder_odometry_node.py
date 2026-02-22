#!/usr/bin/env python3
import math
import threading
from time import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

from gpiozero import DigitalInputDevice


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_odometry')

        # ====== PARÂMETROS (AJUSTE PARA O SEU ROBÔ) ======
        self.declare_parameter('left_gpio', 16)
        self.declare_parameter('right_gpio', 26)

        self.declare_parameter('ticks_per_rev', 20)      # pulsos por volta (do seu encoder)
        self.declare_parameter('wheel_radius', 0.05)     # metros
        self.declare_parameter('wheel_base', 0.30)       # distância entre rodas (m)

        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')

        self.declare_parameter('publish_rate_hz', 30.0)

        # Se você NÃO tem quadratura, escolha:
        #  1) deixar True e mapear só indo pra frente
        #  2) mudar para False e implementar sinal do motor
        self.declare_parameter('assume_forward_only', True)

        # ===============================================

        self.left_gpio = int(self.get_parameter('left_gpio').value)
        self.right_gpio = int(self.get_parameter('right_gpio').value)

        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        self.frame_odom = str(self.get_parameter('frame_odom').value)
        self.frame_base = str(self.get_parameter('frame_base').value)

        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.assume_forward_only = bool(self.get_parameter('assume_forward_only').value)

        # Contadores de pulsos
        self._lock = threading.Lock()
        self._ticks_l = 0
        self._ticks_r = 0

        # Estado do robô (pose integrada)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Encoder GPIO
        self.enc_l = DigitalInputDevice(self.left_gpio, pull_up=True)
        self.enc_r = DigitalInputDevice(self.right_gpio, pull_up=True)

        self.enc_l.when_activated = self._pulse_left
        self.enc_r.when_activated = self._pulse_right

        # ROS pubs
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = time()
        self.last_ticks_l = 0
        self.last_ticks_r = 0

        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self._update)

        self.get_logger().info(
            f'EncoderOdom iniciado. left_gpio={self.left_gpio} right_gpio={self.right_gpio} '
            f'ticks_per_rev={self.ticks_per_rev} wheel_radius={self.wheel_radius} wheel_base={self.wheel_base}'
        )

    def _pulse_left(self):
        with self._lock:
            self._ticks_l += 1

    def _pulse_right(self):
        with self._lock:
            self._ticks_r += 1

    def _update(self):
        now = time()
        dt = now - self.last_time
        if dt <= 0.0:
            return

        with self._lock:
            ticks_l = self._ticks_l
            ticks_r = self._ticks_r

        d_ticks_l = ticks_l - self.last_ticks_l
        d_ticks_r = ticks_r - self.last_ticks_r

        self.last_ticks_l = ticks_l
        self.last_ticks_r = ticks_r
        self.last_time = now

        # Se não tem direção, você só tem magnitude.
        # Para mapear, o mais seguro é mover só para frente enquanto gera o mapa.
        sign_l = 1.0
        sign_r = 1.0
        if not self.assume_forward_only:
            # Aqui você pode aplicar o sinal conforme seu comando de motor.
            # Ex.: sign_l = +1 ou -1; sign_r = +1 ou -1
            pass

        d_ticks_l *= sign_l
        d_ticks_r *= sign_r

        # Distância por tick (m)
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        dl = d_ticks_l * meters_per_tick
        dr = d_ticks_r * meters_per_tick

        # Cinemática diferencial
        ds = (dr + dl) / 2.0
        d_yaw = (dr - dl) / self.wheel_base

        # Integra pose (modelo simples)
        self.yaw += d_yaw
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

        vx = ds / dt
        vth = d_yaw / dt

        # Publica Odometry
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)

        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(vth)

        self.odom_pub.publish(odom)

        # Publica TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_odom
        t.child_frame_id = self.frame_base
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quat(self.yaw)

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = EncoderOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()