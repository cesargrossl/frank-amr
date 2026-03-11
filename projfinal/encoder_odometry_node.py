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


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_odometry')

        # ================== PARÂMETROS ==================
        self.declare_parameter('left_gpio', 16)
        self.declare_parameter('right_gpio', 26)

        # Roda 68 mm -> raio 34 mm = 0.034 m
        self.declare_parameter('wheel_radius', 0.034)

        # Medido no seu teste
        self.declare_parameter('ticks_per_rev', 22)

        # Ajuste conforme medida real entre centros das rodas
        self.declare_parameter('wheel_base', 0.15)

        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')

        self.declare_parameter('publish_rate_hz', 30.0)

        # Como seu encoder não informa direção sozinho,
        # o mais seguro é mapear inicialmente só andando para frente
        self.declare_parameter('assume_forward_only', True)

        # Pequeno debounce para evitar ruído
        self.declare_parameter('bounce_time', 0.0015)

        # Intervalo de log
        self.declare_parameter('debug_log_every_sec', 1.0)
        # =================================================

        self.left_gpio = int(self.get_parameter('left_gpio').value)
        self.right_gpio = int(self.get_parameter('right_gpio').value)

        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)

        self.frame_odom = str(self.get_parameter('frame_odom').value)
        self.frame_base = str(self.get_parameter('frame_base').value)

        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.assume_forward_only = bool(self.get_parameter('assume_forward_only').value)
        self.bounce_time = float(self.get_parameter('bounce_time').value)
        self.debug_log_every_sec = float(self.get_parameter('debug_log_every_sec').value)

        if self.ticks_per_rev <= 0:
            raise ValueError('ticks_per_rev deve ser > 0')
        if self.wheel_radius <= 0:
            raise ValueError('wheel_radius deve ser > 0')
        if self.wheel_base <= 0:
            raise ValueError('wheel_base deve ser > 0')

        self._lock = threading.Lock()
        self._ticks_l = 0
        self._ticks_r = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.enc_l = DigitalInputDevice(
            self.left_gpio,
            pull_up=True,
            bounce_time=self.bounce_time
        )
        self.enc_r = DigitalInputDevice(
            self.right_gpio,
            pull_up=True,
            bounce_time=self.bounce_time
        )

        self.enc_l.when_activated = self._pulse_left
        self.enc_r.when_activated = self._pulse_right

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = time()
        self.last_ticks_l = 0
        self.last_ticks_r = 0
        self.last_debug_time = time()

        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self._update)

        self.get_logger().info(
            'EncoderOdom iniciado | '
            f'left_gpio={self.left_gpio} right_gpio={self.right_gpio} | '
            f'ticks_per_rev={self.ticks_per_rev} | '
            f'wheel_radius={self.wheel_radius:.4f} m | '
            f'wheel_base={self.wheel_base:.4f} m | '
            f'bounce_time={self.bounce_time:.4f} s | '
            f'assume_forward_only={self.assume_forward_only}'
        )

    def _pulse_left(self):
        with self._lock:
            self._ticks_l += 1

    def _pulse_right(self):
        with self._lock:
            self._ticks_r += 1

    def _get_wheel_signs(self):
        sign_l = 1.0
        sign_r = 1.0

        if not self.assume_forward_only:
            # Futuramente você pode integrar o sinal dos motores aqui
            pass

        return sign_l, sign_r

    def _update(self):
        now = time()
        dt = now - self.last_time

        if dt <= 0.0 or dt < 1e-4:
            return

        with self._lock:
            ticks_l = self._ticks_l
            ticks_r = self._ticks_r

        d_ticks_l = ticks_l - self.last_ticks_l
        d_ticks_r = ticks_r - self.last_ticks_r

        self.last_ticks_l = ticks_l
        self.last_ticks_r = ticks_r
        self.last_time = now

        sign_l, sign_r = self._get_wheel_signs()
        d_ticks_l *= sign_l
        d_ticks_r *= sign_r

        # distância linear percorrida por 1 pulso
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        dl = d_ticks_l * meters_per_tick
        dr = d_ticks_r * meters_per_tick

        ds = (dr + dl) / 2.0
        d_yaw = (dr - dl) / self.wheel_base

        yaw_mid = self.yaw + (d_yaw / 2.0)
        self.x += ds * math.cos(yaw_mid)
        self.y += ds * math.sin(yaw_mid)
        self.yaw = normalize_angle(self.yaw + d_yaw)

        vx = ds / dt
        vth = d_yaw / dt

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

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05

        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.frame_odom
        t.child_frame_id = self.frame_base
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation = yaw_to_quat(self.yaw)

        self.tf_broadcaster.sendTransform(t)

        if (now - self.last_debug_time) >= self.debug_log_every_sec:
            self.last_debug_time = now
            self.get_logger().info(
                f'ticks L/R=({ticks_l},{ticks_r}) | '
                f'delta L/R=({int(d_ticks_l)},{int(d_ticks_r)}) | '
                f'm_per_tick={meters_per_tick:.5f} | '
                f'dl={dl:.4f} m dr={dr:.4f} m | '
                f'x={self.x:.3f} y={self.y:.3f} yaw={self.yaw:.3f}'
            )


def main():
    rclpy.init()
    node = EncoderOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()