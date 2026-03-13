#!/usr/bin/env python3
import sys
import termios
import tty
import select
import time

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray


class KeyboardMotorController(Node):
    def __init__(self):
        super().__init__('keyboard_motor_controller')

        # =============================
        # PINOS DOS MOTORES
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

        self.pub_dir = self.create_publisher(Int8MultiArray, '/wheel_dir', 10)

        self.stop_timeout = 0.18
        self.last_cmd = None
        self.last_key_time = 0.0

        self.get_logger().info('Controle do robô iniciado')
        self.get_logger().info('Segure W/A/S/D para mover')
        self.get_logger().info('Espaço = parar | Q = sair')

    def publish_wheel_dir(self, left_sign: int, right_sign: int):
        msg = Int8MultiArray()
        msg.data = [int(left_sign), int(right_sign)]
        self.pub_dir.publish(msg)

    def parar(self):
        GPIO.output(self.IN1, 0)
        GPIO.output(self.IN2, 0)
        GPIO.output(self.IN3, 0)
        GPIO.output(self.IN4, 0)
        self.publish_wheel_dir(0, 0)

    def frente(self):
        GPIO.output(self.IN1, 1)
        GPIO.output(self.IN2, 0)
        GPIO.output(self.IN3, 1)
        GPIO.output(self.IN4, 0)
        self.publish_wheel_dir(+1, +1)

    def re(self):
        GPIO.output(self.IN1, 0)
        GPIO.output(self.IN2, 1)
        GPIO.output(self.IN3, 0)
        GPIO.output(self.IN4, 1)
        self.publish_wheel_dir(-1, -1)

    def esquerda(self):
        GPIO.output(self.IN1, 0)
        GPIO.output(self.IN2, 1)
        GPIO.output(self.IN3, 1)
        GPIO.output(self.IN4, 0)
        self.publish_wheel_dir(-1, +1)

    def direita(self):
        GPIO.output(self.IN1, 1)
        GPIO.output(self.IN2, 0)
        GPIO.output(self.IN3, 0)
        GPIO.output(self.IN4, 1)
        self.publish_wheel_dir(+1, -1)

    @staticmethod
    def get_key(timeout=0.04):
        dr, _, _ = select.select([sys.stdin], [], [], timeout)
        if dr:
            return sys.stdin.read(1)
        return None

    def run(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setcbreak(fd)

            while rclpy.ok():
                key = self.get_key()
                now = time.monotonic()

                if key is not None:
                    key = key.lower()

                    if key == 'w':
                        self.frente()
                        self.last_cmd = 'w'
                        self.last_key_time = now

                    elif key == 's':
                        self.re()
                        self.last_cmd = 's'
                        self.last_key_time = now

                    elif key == 'a':
                        self.esquerda()
                        self.last_cmd = 'a'
                        self.last_key_time = now

                    elif key == 'd':
                        self.direita()
                        self.last_cmd = 'd'
                        self.last_key_time = now

                    elif key == ' ':
                        self.parar()
                        self.last_cmd = None

                    elif key == 'q':
                        break

                if self.last_cmd is not None and (now - self.last_key_time) > self.stop_timeout:
                    self.parar()
                    self.last_cmd = None

                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.01)

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.parar()
            GPIO.cleanup()


def main():
    rclpy.init()
    node = KeyboardMotorController()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()