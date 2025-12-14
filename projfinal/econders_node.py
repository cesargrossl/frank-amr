#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from gpiozero import DigitalInputDevice


class WheelEncoders(Node):
    def __init__(self):
        super().__init__('wheel_encoders')

        # ====== PARAMETROS ======
        self.declare_parameter('gpio_left', 16)
        self.declare_parameter('gpio_right', 26)
        self.declare_parameter('ticks_per_rev', 20)          # furos do disco
        self.declare_parameter('wheel_radius_m', 0.03)       # raio da roda em metros
        self.declare_parameter('wheel_base_m', 0.18)         # distância entre rodas (centro a centro)
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('frame_odom', 'odom')
        self.declare_parameter('frame_base', 'base_link')

        self.gpio_left = int(self.get_parameter('gpio_left').value)
        self.gpio_right = int(self.get_parameter('gpio_right').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base = float(self.get_parameter('wheel_base_m').value)
        self.publish_odom = bool(self.get_parameter('publish_odom').value)
        self.frame_odom = str(self.get_parameter('frame_odom').value)
        self.frame_base = str(self.get_parameter('frame_base').value)

        # ====== GPIO ======
        self.enc_left = DigitalInputDevice(self.gpio_left)
        self.enc_right = DigitalInputDevice(self.gpio_right)

        self.left_ticks = 0
        self.right_ticks = 0

        def left_cb():
            self.left_ticks += 1

        def right_cb():
            self.right_ticks += 1

        self.enc_left.when_activated = left_cb
        self.enc_right.when_activated = right_cb

        # ====== PUB ======
        self.pub_l = self.create_publisher(Int32, '/encoder/left_ticks', 10)
        self.pub_r = self.create_publisher(Int32, '/encoder/right_ticks', 10)

        self.pub_odom = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ====== ODOM STATE ======
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.prev_l = 0
        self.prev_r = 0

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz

        self.get_logger().info(
            f'Encoders OK. L GPIO={self.gpio_left}, R GPIO={self.gpio_right}, publish_odom={self.publish_odom}'
        )

    def loop(self):
        # publica ticks
        ml = Int32(); ml.data = int(self.left_ticks)
        mr = Int32(); mr.data = int(self.right_ticks)
        self.pub_l.publish(ml)
        self.pub_r.publish(mr)

        if not self.publish_odom:
            return

        # calcula delta ticks
        dl_ticks = self.left_ticks - self.prev_l
        dr_ticks = self.right_ticks - self.prev_r
        self.prev_l = self.left_ticks
        self.prev_r = self.right_ticks

        # tempo
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        # ticks -> distancia (m)
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        dl = dl_ticks * meters_per_tick
        dr = dr_ticks * meters_per_tick

        # cinemática diferencial
        ds = (dr + dl) / 2.0
        dtheta = (dr - dl) / self.wheel_base

        # integra pose
        self.yaw += dtheta
        self.x += ds * math.cos(self.yaw)
        self.y += ds * math.sin(self.yaw)

        # velocidades
        v = ds / dt
        w = dtheta / dt

        # ODOM msg
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_base
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # yaw -> quaternion
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.pub_odom.publish(odom)

        # TF odom -> base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.frame_odom
        t.child_frame_id = self.frame_base
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = WheelEncoders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
#chmod +x ~/ros2_ws/src/wheel_encoders/wheel_encoders/encoders_node.py
