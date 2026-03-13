#!/usr/bin/env python3
import RPi.GPIO as GPIO
import sys
import termios
import tty
import select
import time

# =============================
# PINOS DOS MOTORES
# =============================
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# =============================
# MOVIMENTOS
# =============================
def parar():
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 0)
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 0)

def frente():
    GPIO.output(IN1, 1)
    GPIO.output(IN2, 0)
    GPIO.output(IN3, 1)
    GPIO.output(IN4, 0)

def re():
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 1)
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 1)

def esquerda():
    GPIO.output(IN1, 0)
    GPIO.output(IN2, 1)
    GPIO.output(IN3, 1)
    GPIO.output(IN4, 0)

def direita():
    GPIO.output(IN1, 1)
    GPIO.output(IN2, 0)
    GPIO.output(IN3, 0)
    GPIO.output(IN4, 1)

# =============================
# LEITURA NÃO BLOQUEANTE
# =============================
def get_key(timeout=0.02):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def main():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    # tempo máximo sem receber repetição da tecla
    # depois disso ele para
    STOP_TIMEOUT = 0.18

    last_cmd = None
    last_key_time = 0.0

    print("Controle do robô")
    print("Segure W/A/S/D para mover")
    print("Soltou a tecla -> para sozinho")
    print("Espaço = parar")
    print("Q = sair")

    try:
        tty.setcbreak(fd)

        while True:
            key = get_key()
            now = time.monotonic()

            if key is not None:
                key = key.lower()

                if key == 'w':
                    frente()
                    last_cmd = 'w'
                    last_key_time = now

                elif key == 's':
                    re()
                    last_cmd = 's'
                    last_key_time = now

                elif key == 'a':
                    esquerda()
                    last_cmd = 'a'
                    last_key_time = now

                elif key == 'd':
                    direita()
                    last_cmd = 'd'
                    last_key_time = now

                elif key == ' ':
                    parar()
                    last_cmd = None

                elif key == 'q':
                    break

            # Se parou de chegar repetição da tecla, para o robô
            if last_cmd is not None and (now - last_key_time) > STOP_TIMEOUT:
                parar()
                last_cmd = None

            time.sleep(0.01)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        parar()
        GPIO.cleanup()

if __name__ == "__main__":
    main()