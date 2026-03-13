#!/usr/bin/env python3
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO


class TB6612CmdVel(Node):
    def __init__(self):
        super().__init__('tb6612_cmdvel_node')

        # ======= Parâmetros (ajuste para sua fiação) =======
        # Entradas de direção
        self.declare_parameter('ain1', 17)
        self.declare_parameter('ain2', 27)
        self.declare_parameter('bin1', 22)
        self.declare_parameter('bin2', 23)

        # PWM (velocidade) - TB6612FNG tem PWMA e PWMB
        self.declare_parameter('pwma', 18)   # GPIO18 (PWM0) recomendado
        self.declare_parameter('pwmb', 13)   # GPIO13 (PWM1) recomendado

        # Standby
        self.declare_parameter('stby', 5)    # pode ser qualquer GPIO

        # Limites e geometria do robô
        self.declare_parameter('wheel_base', 0.20)       # distância entre rodas (m) - AJUSTE
        self.declare_parameter('max_lin', 0.30)          # m/s - AJUSTE
        self.declare_parameter('max_ang', 1.50)          # rad/s - AJUSTE

        # Segurança
        self.declare_parameter('cmd_timeout', 0.5)       # s
        self.declare_parameter('min_pwm', 25.0)          # % mínimo p/ vencer atrito (AJUSTE)
        self.declare_parameter('pwm_freq', 1500)         # Hz

        self.ain1 = int(self.get_parameter('ain1').value)
        self.ain2 = int(self.get_parameter('ain2').value)
        self.bin1 = int(self.get_parameter('bin1').value)
        self.bin2 = int(self.get_parameter('bin2').value)
        self.pwma_pin = int(self.get_parameter('pwma').value)
        self.pwmb_pin = int(self.get_parameter('pwmb').value)
        self.stby = int(self.get_parameter('stby').value)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)

        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.min_pwm = float(self.get_parameter('min_pwm').value)
        self.pwm_freq = float(self.get_parameter('pwm_freq').value)

        # ======= GPIO =======
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for p in [self.ain1, self.ain2, self.bin1, self.bin2, self.stby, self.pwma_pin, self.pwmb_pin]:
            GPIO.setup(p, GPIO.OUT)

        GPIO.output(self.stby, 1)  # habilita driver

        self.pwma = GPIO.PWM(self.pwma_pin, self.pwm_freq)
        self.pwmb = GPIO.PWM(self.pwmb_pin, self.pwm_freq)
        self.pwma.start(0.0)
        self.pwmb.start(0.0)

        self.last_cmd_time = time.time()

        # Subscriber
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)

        # Timer de segurança
        self.timer = self.create_timer(0.05, self.watchdog)

        self.get_logger().info("TB6612CmdVel pronto. Escutando /cmd_vel ...")

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def set_motor(self, in1, in2, pwm, speed):
        """
        speed em [-1..1]
        """
        if abs(speed) < 1e-3:
            GPIO.output(in1, 0)
            GPIO.output(in2, 0)
            pwm.ChangeDutyCycle(0.0)
            return

        direction = 1 if speed > 0 else -1
        duty = abs(speed) * 100.0

        # mínimo p/ vencer atrito
        if 0.0 < duty < self.min_pwm:
            duty = self.min_pwm

        duty = self.clamp(duty, 0.0, 100.0)

        if direction > 0:
            GPIO.output(in1, 1)
            GPIO.output(in2, 0)
        else:
            GPIO.output(in1, 0)
            GPIO.output(in2, 1)

        pwm.ChangeDutyCycle(duty)

    def stop_all(self):
        GPIO.output(self.ain1, 0); GPIO.output(self.ain2, 0)
        GPIO.output(self.bin1, 0); GPIO.output(self.bin2, 0)
        self.pwma.ChangeDutyCycle(0.0)
        self.pwmb.ChangeDutyCycle(0.0)

    def on_cmd(self, msg: Twist):
        self.last_cmd_time = time.time()

        # Nav2 manda linear.x e angular.z
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # limita
        v = self.clamp(v, -self.max_lin, self.max_lin)
        w = self.clamp(w, -self.max_ang, self.max_ang)

        # diferencial: v_left = v - w*L/2 ; v_right = v + w*L/2
        vl = v - (w * self.wheel_base / 2.0)
        vr = v + (w * self.wheel_base / 2.0)

        # normaliza para [-1..1] usando max_lin como base
        if self.max_lin < 1e-6:
            self.stop_all()
            return

        sl = self.clamp(vl / self.max_lin, -1.0, 1.0)
        sr = self.clamp(vr / self.max_lin, -1.0, 1.0)

        self.set_motor(self.ain1, self.ain2, self.pwma, sl)
        self.set_motor(self.bin1, self.bin2, self.pwmb, sr)

    def watchdog(self):
        if (time.time() - self.last_cmd_time) > self.cmd_timeout:
            self.stop_all()

    def destroy_node(self):
        try:
            self.stop_all()
            GPIO.output(self.stby, 0)
            self.pwma.stop()
            self.pwmb.stop()
            GPIO.cleanup()
        except Exception:
            pass
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