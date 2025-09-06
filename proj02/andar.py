# l298n_pi.py — Controle de 2 motores com L298N (Raspberry Pi)
# Sequência: frente -> trás -> giro (A frente, B trás)

import time
import RPi.GPIO as GPIO

# ==== Mapeamento dos pinos (BCM) ====
IN1 = 17  # Motor A
IN2 = 27  # Motor A
IN3 = 22  # Motor B
IN4 = 23  # Motor B

T = 3.0   # tempo em segundos

def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in (IN1, IN2, IN3, IN4):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

def frente():
    # A: IN1=H, IN2=L | B: IN3=H, IN4=L
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def tras():
    # A: IN1=L, IN2=H | B: IN3=L, IN4=H
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def girar():
    # A frente, B trás
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def parar():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def loop():
    while True:
        # 1) Ambos pra frente
        frente()
        time.sleep(T)

        parar()
        time.sleep(1.0)

        # 2) Ambos pra trás
        tras()
        time.sleep(T)

        parar()
        time.sleep(1.0)

        # 3) Giro
        girar()
        time.sleep(T)

        parar()
        time.sleep(2.0)

def main():
    setup()
    try:
        loop()
    except KeyboardInterrupt:
        pass
    finally:
        parar()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
