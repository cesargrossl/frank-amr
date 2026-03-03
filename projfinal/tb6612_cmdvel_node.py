#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO


class TB6612CmdVelGPIO(Node):
    def __init__(self):
        super().__init__("tb6612_cmdvel_gpio")

        # ===== AJUSTE PINOS (BCM) =====
        IN1 = 17  # Motor A
        IN2 = 27
        IN3 = 22  # Motor B
        IN4 = 23

        # PWM (se você ligou PWMA/PWMB em GPIO). Se NÃO ligou, deixe None.
        PWMA = None   # ex: 18
        PWMB = None   # ex: 13

        # STBY (se você ligou em GPIO). Se ligou direto no 3.3V, deixe None.
        STBY = None   # ex: 24

        # Inversões (se algum lado estiver invertido)
        INV_A = True
        INV_B = True

        # Geometria / limites
        WHEEL_BASE = 0.20   # metros (distância entre rodas) -> ajuste
        MAX_V = 0.25        # m/s
        MAX_W = 1.5         # rad/s

        PWM_HZ = 200        # RPi.GPIO PWM é melhor em frequências mais baixas
        CMD_TIMEOUT = 0.4   # s

        # =================================

        self.IN1, self.IN2, self.IN3, self.IN4 = IN1, IN2, IN3, IN4
        self.PWMA, self.PWMB = PWMA, PWMB
        self.STBY = STBY
        self.INV_A, self.INV_B = INV_A, INV_B
        self.WHEEL_BASE = WHEEL_BASE
        self.MAX_V = MAX_V
        self.MAX_W = MAX_W
        self.PWM_HZ = PWM_HZ
        self.CMD_TIMEOUT = CMD_TIMEOUT

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for p in [IN1, IN2, IN3, IN4]:
            GPIO.setup(p, GPIO.OUT)
            GPIO.output(p, 0)

        if STBY is not None:
            GPIO.setup(STBY, GPIO.OUT)
            GPIO.output(STBY, 1)

        self.pwm_a = None
        self.pwm_b = None
        if PWMA is not None:
            GPIO.setup(PWMA, GPIO.OUT)
            self.pwm_a = GPIO.PWM(PWMA, PWM_HZ)
            self.pwm_a.start(0)
        if PWMB is not None:
            GPIO.setup(PWMB, GPIO.OUT)
            self.pwm_b = GPIO.PWM(PWMB, PWM_HZ)
            self.pwm_b.start(0)

        self.last_cmd = time.time()

        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(0.05, self.watchdog)

        self.get_logger().info("OK: escutando /cmd_vel (TB6612 via RPi.GPIO)")

    def clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def set_dir_pwm(self, in1, in2, pwm, value, inv=False):
        # value: -1..+1
        if inv:
            value = -value

        if abs(value) < 0.02:
            GPIO.output(in1, 0); GPIO.output(in2, 0)
            if pwm is not None:
                pwm.ChangeDutyCycle(0)
            return

        if value > 0:
            GPIO.output(in1, 1); GPIO.output(in2, 0)
        else:
            GPIO.output(in1, 0); GPIO.output(in2, 1)

        duty = int(self.clamp(abs(value) * 100.0, 0.0, 100.0))
        if pwm is not None:
            pwm.ChangeDutyCycle(duty)
        # Se PWM não existe, assume PWMA/PWMB estão em HIGH físico (velocidade máxima)

    def stop_all(self):
        self.set_dir_pwm(self.IN1, self.IN2, self.pwm_a, 0.0, inv=False)
        self.set_dir_pwm(self.IN3, self.IN4, self.pwm_b, 0.0, inv=False)

    def on_cmd(self, msg: Twist):
        self.last_cmd = time.time()

        v = self.clamp(msg.linear.x, -self.MAX_V, self.MAX_V)
        w = self.clamp(msg.angular.z, -self.MAX_W, self.MAX_W)

        v_left  = v - (w * self.WHEEL_BASE / 2.0)
        v_right = v + (w * self.WHEEL_BASE / 2.0)

        left_norm = self.clamp(v_left / self.MAX_V if self.MAX_V > 0 else 0.0, -1.0, 1.0)
        right_norm = self.clamp(v_right / self.MAX_V if self.MAX_V > 0 else 0.0, -1.0, 1.0)

        self.set_dir_pwm(self.IN1, self.IN2, self.pwm_a, left_norm, inv=self.INV_A)
        self.set_dir_pwm(self.IN3, self.IN4, self.pwm_b, right_norm, inv=self.INV_B)

    def watchdog(self):
        if (time.time() - self.last_cmd) > self.CMD_TIMEOUT:
            self.stop_all()

    def destroy_node(self):
        try:
            self.stop_all()
            if self.pwm_a is not None:
                self.pwm_a.stop()
            if self.pwm_b is not None:
                self.pwm_b.stop()
            if self.STBY is not None:
                GPIO.output(self.STBY, 0)
            GPIO.cleanup()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = TB6612CmdVelGPIO()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()