#!/usr/bin/env python3
import select
import sys
import termios
import time
import tty

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray


class KeyboardMotorController(Node):
    def __init__(self):
        super().__init__('keyboard_motor_controller')

        # Ponte H / GPIOs
        self.LEFT_IN1 = 17
        self.LEFT_IN2 = 27
        self.RIGHT_IN1 = 22
        self.RIGHT_IN2 = 23

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.LEFT_IN1, GPIO.OUT)
        GPIO.setup(self.LEFT_IN2, GPIO.OUT)
        GPIO.setup(self.RIGHT_IN1, GPIO.OUT)
        GPIO.setup(self.RIGHT_IN2, GPIO.OUT)

        self.pub_dir = self.create_publisher(Int8MultiArray, '/wheel_dir', 10)

        self.stop_timeout = 0.18
        self.last_cmd = None
        self.last_key_time = 0.0

        # Mantido apenas como metadado para a odometria.
        # Sem PWM real, usamos apenas o "modo" de curva.
        self.turn_mode = 'pivot'  # pivot = um lado anda / outro para
        self.allow_spin_in_place = False

        self.get_logger().info('Controle do robô iniciado')
        self.get_logger().info('W = frente | S = ré | A = esquerda suave | D = direita suave')
        self.get_logger().info('Z = gira no eixo à esquerda | C = gira no eixo à direita')
        self.get_logger().info('Espaço = parar | Q = sair')

    def publish_wheel_dir(self, left_sign: int, right_sign: int):
        msg = Int8MultiArray()
        msg.data = [int(left_sign), int(right_sign)]
        self.pub_dir.publish(msg)

    def lado_esquerdo_frente(self):
        GPIO.output(self.LEFT_IN1, GPIO.LOW)
        GPIO.output(self.LEFT_IN2, GPIO.HIGH)

    def lado_esquerdo_re(self):
        GPIO.output(self.LEFT_IN1, GPIO.HIGH)
        GPIO.output(self.LEFT_IN2, GPIO.LOW)

    def lado_direito_frente(self):
        GPIO.output(self.RIGHT_IN1, GPIO.LOW)
        GPIO.output(self.RIGHT_IN2, GPIO.HIGH)

    def lado_direito_re(self):
        GPIO.output(self.RIGHT_IN1, GPIO.HIGH)
        GPIO.output(self.RIGHT_IN2, GPIO.LOW)

    def parar_lado_esquerdo(self):
        GPIO.output(self.LEFT_IN1, GPIO.LOW)
        GPIO.output(self.LEFT_IN2, GPIO.LOW)

    def parar_lado_direito(self):
        GPIO.output(self.RIGHT_IN1, GPIO.LOW)
        GPIO.output(self.RIGHT_IN2, GPIO.LOW)

    def parar(self):
        self.parar_lado_esquerdo()
        self.parar_lado_direito()
        self.publish_wheel_dir(0, 0)

    def frente(self):
        self.lado_esquerdo_frente()
        self.lado_direito_frente()
        self.publish_wheel_dir(+1, +1)

    def re(self):
        self.lado_esquerdo_re()
        self.lado_direito_re()
        self.publish_wheel_dir(-1, -1)

    def esquerda(self):
        # Curva suave para reduzir patinagem:
        # lado esquerdo para, lado direito anda.
        self.parar_lado_esquerdo()
        self.lado_direito_frente()
        self.publish_wheel_dir(0, +1)

    def direita(self):
        # Curva suave para reduzir patinagem:
        # lado direito para, lado esquerdo anda.
        self.lado_esquerdo_frente()
        self.parar_lado_direito()
        self.publish_wheel_dir(+1, 0)

    def giro_eixo_esquerda(self):
        if not self.allow_spin_in_place:
            self.get_logger().warn(
                'Giro no eixo desabilitado por padrão para reduzir derrapagem. '
                'Usando curva suave para a esquerda.'
            )
            self.esquerda()
            return

        self.lado_esquerdo_re()
        self.lado_direito_frente()
        self.publish_wheel_dir(-1, +1)

    def giro_eixo_direita(self):
        if not self.allow_spin_in_place:
            self.get_logger().warn(
                'Giro no eixo desabilitado por padrão para reduzir derrapagem. '
                'Usando curva suave para a direita.'
            )
            self.direita()
            return

        self.lado_esquerdo_frente()
        self.lado_direito_re()
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

                    elif key == 'z':
                        self.giro_eixo_esquerda()
                        self.last_cmd = 'z'
                        self.last_key_time = now

                    elif key == 'c':
                        self.giro_eixo_direita()
                        self.last_cmd = 'c'
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
                time.sleep(0.05)

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
