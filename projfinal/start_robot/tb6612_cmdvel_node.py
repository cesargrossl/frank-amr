#!/usr/bin/env python3
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray


class TB6612CmdVel(Node):
    def __init__(self):
        super().__init__('tb6612_cmdvel_node')

        # =============================
        # PINOS TB6612
        # =============================
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

        # =============================
        # PARÂMETROS DE CONTROLE
        # =============================
        self.linear_deadband = 0.02
        self.angular_deadband = 0.05
        self.cmd_timeout = 0.5

        # =============================
        # ESTADO ATUAL
        # =============================
        self.last_cmd_time = self.get_clock().now()
        self.last_left_dir = 0
        self.last_right_dir = 0

        # =============================
        # ROS2 PUB/SUB
        # =============================
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.dir_pub = self.create_publisher(
            Int8MultiArray,
            '/wheel_dir',
            10
        )

        self.timer = self.create_timer(0.1, self.safety_stop)

        self.get_logger().info('TB6612 pronto, ouvindo /cmd_vel e publicando /wheel_dir')

    # =========================================================
    # CONTROLE DOS MOTORES
    # =========================================================
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

    def publish_wheel_dir(self, left_dir: int, right_dir: int):
        msg = Int8MultiArray()
        msg.data = [int(left_dir), int(right_dir)]
        self.dir_pub.publish(msg)

    def stop_all(self):
        GPIO.output(self.IN1, 0)
        GPIO.output(self.IN2, 0)
        GPIO.output(self.IN3, 0)
        GPIO.output(self.IN4, 0)

        self.last_left_dir = 0
        self.last_right_dir = 0
        self.publish_wheel_dir(0, 0)

    # =========================================================
    # CALLBACK CMD_VEL
    # =========================================================
    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        v = msg.linear.x
        w = msg.angular.z

        # Deadband
        if abs(v) < self.linear_deadband:
            v = 0.0
        if abs(w) < self.angular_deadband:
            w = 0.0

        # Parado
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

        # Aplica nos motores
        self.set_left_motor(left_dir)
        self.set_right_motor(right_dir)

        # Publica direção das rodas para a odometria
        self.last_left_dir = left_dir
        self.last_right_dir = right_dir
        self.publish_wheel_dir(left_dir, right_dir)

        self.get_logger().info(
            f'/cmd_vel recebido | v={v:.3f} w={w:.3f} | '
            f'left={left:.3f} right={right:.3f} | '
            f'dir=({left_dir},{right_dir})'
        )

    # =========================================================
    # PARADA POR SEGURANÇA
    # =========================================================
    def safety_stop(self):
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_timeout:
            if self.last_left_dir != 0 or self.last_right_dir != 0:
                self.get_logger().warn('Timeout em /cmd_vel, parando motores')
            self.stop_all()

    # =========================================================
    # FINALIZAÇÃO
    # =========================================================
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