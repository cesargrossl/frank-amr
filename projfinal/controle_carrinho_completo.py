#!/usr/bin/env python3
import RPi.GPIO as GPIO
import sys, termios, tty, select, time

# Motores do carrinho
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23

# Motor do garfo - AJUSTE se usar outros GPIOs
GARFO_IN1 = 5
GARFO_IN2 = 6

# Fins de curso
FIM_SUPERIOR = 21
FIM_INFERIOR = 20

GPIO.setmode(GPIO.BCM)

for pino in [IN1, IN2, IN3, IN4, GARFO_IN1, GARFO_IN2]:
    GPIO.setup(pino, GPIO.OUT)

GPIO.setup(FIM_SUPERIOR, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(FIM_INFERIOR, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def fim_superior_acionado():
    return GPIO.input(FIM_SUPERIOR) == GPIO.HIGH

def fim_inferior_acionado():
    return GPIO.input(FIM_INFERIOR) == GPIO.HIGH

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

def parar_garfo():
    GPIO.output(GARFO_IN1, 0)
    GPIO.output(GARFO_IN2, 0)

def subir_garfo():
    if fim_superior_acionado():
        print("Fim superior acionado. Garfo parado.")
        parar_garfo()
    else:
        GPIO.output(GARFO_IN1, 1)
        GPIO.output(GARFO_IN2, 0)

def descer_garfo():
    if fim_inferior_acionado():
        print("Fim inferior acionado. Garfo parado.")
        parar_garfo()
    else:
        GPIO.output(GARFO_IN1, 0)
        GPIO.output(GARFO_IN2, 1)

def get_key(timeout=0.04):
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def main():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    STOP_TIMEOUT = 0.18
    last_mov = None
    last_garfo = None
    last_mov_time = 0
    last_garfo_time = 0

    print("Controle do robô")
    print("W/A/S/D = carrinho")
    print("P = subir garfo")
    print("L = descer garfo")
    print("Espaço = parar tudo")
    print("Q = sair")

    try:
        tty.setcbreak(fd)

        while True:
            key = get_key()
            now = time.monotonic()

            if key:
                key = key.lower()

                if key == 'w':
                    frente()
                    last_mov = 'w'
                    last_mov_time = now

                elif key == 's':
                    re()
                    last_mov = 's'
                    last_mov_time = now

                elif key == 'a':
                    esquerda()
                    last_mov = 'a'
                    last_mov_time = now

                elif key == 'd':
                    direita()
                    last_mov = 'd'
                    last_mov_time = now

                elif key == 'p':
                    subir_garfo()
                    last_garfo = 'p'
                    last_garfo_time = now

                elif key == 'l':
                    descer_garfo()
                    last_garfo = 'l'
                    last_garfo_time = now

                elif key == ' ':
                    parar()
                    parar_garfo()
                    last_mov = None
                    last_garfo = None

                elif key == 'q':
                    break

            # Para carrinho se soltar tecla
            if last_mov and (now - last_mov_time) > STOP_TIMEOUT:
                parar()
                last_mov = None

            # Para garfo se soltar tecla
            if last_garfo and (now - last_garfo_time) > STOP_TIMEOUT:
                parar_garfo()
                last_garfo = None

            # Segurança: para garfo ao bater no fim de curso
            if last_garfo == 'p' and fim_superior_acionado():
                parar_garfo()
                last_garfo = None
                print("Garfo chegou no fim superior.")

            if last_garfo == 'l' and fim_inferior_acionado():
                parar_garfo()
                last_garfo = None
                print("Garfo chegou no fim inferior.")

            time.sleep(0.01)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        parar()
        parar_garfo()
        GPIO.cleanup()

if __name__ == "__main__":
    main()