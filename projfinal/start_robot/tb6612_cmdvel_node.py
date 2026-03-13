#!/usr/bin/env python3
import math
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TB6612CmdVel(Node):
    def __init__(self):
        super().__init__('tb6612_cmdvel_node')

        self.IN1 = 17
        self.IN2 = 27
        self.IN3 = 22
        self.IN4 = 23

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.IN1, GPIO.OUT)
        GPIO.setup(self.IN2, GPIO.OUT)
        GPIO.setup(self.IN3, GPIO.OUT)
        GPIO.setup(self.IN4, GPIO.OUT)

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        self.timer = self.create_timer(0.2, self.safety_stop)
        self.last_cmd_time = self.get_clock().now()

        self.linear_deadband = 0.02
        self.angular_deadband = 0.05

        self.get_logger().info('TB6612 pronto, ouvindo /cmd_vel')

    def set_left_motor(self, direction: int):
        if direction > 0:
            GPIO.output(self.IN1, 1)
            GPIO.output(self.IN2, 0)
        elif direction < 0:
            GPIO.output(self.IN1, 0)
            GPIO.output(self.IN2, 1)
        else:
            GPIO.output(self.IN1, 0)
            GPIO.output(self.IN2, 0)

    def set_right_motor(self, direction: int):
        if direction > 0:
            GPIO.output(self.IN3, 1)
            GPIO.output(self.IN4, 0)
        elif direction < 0:
            GPIO.output(self.IN3, 0)
            GPIO.output(self.IN4, 1)
        else:
            GPIO.output(self.IN3, 0)
            GPIO.output(self.IN4, 0)

    def stop_all(self):
        GPIO.output(self.IN1, 0)
        GPIO.output(self.IN2, 0)
        GPIO.output(self.IN3, 0)
        GPIO.output(self.IN4, 0)

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        v = msg.linear.x
        w = msg.angular.z

        if abs(v) < self.linear_deadband:
            v = 0.0
        if abs(w) < self.angular_deadband:
            w = 0.0

        if v == 0.0 and w == 0.0:
            self.stop_all()
            return

        # Mistura diferencial simples
        left = v - w
        right = v + w

        left_dir = 0
        right_dir = 0

        if left > 0.0:
            left_dir = 1
        elif left < 0.0:
            left_dir = -1

        if right > 0.0:
            right_dir = 1
        elif right < 0.0:
            right_dir = -1

        self.set_left_motor(left_dir)
        self.set_right_motor(right_dir)

    def safety_stop(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > 0.5:
            self.stop_all()

    def destroy_node(self):
        self.stop_all()
        GPIO.cleanup()
        super().destroy_node()

def main():
        rclpy.init()
        node = TB6612CmdVel()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()