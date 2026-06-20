#!/usr/bin/env python3
import RPi.GPIO as GPIO
import sys, termios, tty, select, time

IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23

GARFO_IN1 = 5
GARFO_IN2 = 6

FIM_SUPERIOR = 21
FIM_INFERIOR = 20

GPIO.setmode(GPIO.BCM)

for pino in [IN1, IN2, IN3, IN4, GARFO_IN1, GARFO_IN2]:
    GPIO.setup(pino, GPIO.OUT)

GPIO.setup(FIM_SUPERIOR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(FIM_INFERIOR, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def fim_superior_acionado():
    return GPIO.input(FIM_SUPERIOR) == GPIO.HIGH

def fim_inferior_acionado():
    return GPIO.input(FIM_INFERIOR) == GPIO.HIGH

def parar_carrinho():
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
        print("Bloqueado: fim superior acionado.")
        parar_garfo()
        return False

    GPIO.output(GARFO_IN1, 1)
    GPIO.output(GARFO_IN2, 0)
    return True

def descer_garfo():
    if fim_inferior_acionado():
        print("Bloqueado: fim inferior acionado.")
        parar_garfo()
        return False

    GPIO.output(GARFO_IN1, 0)
    GPIO.output(GARFO_IN2, 1)
    return True

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
    last_mov_time = 0.0
    last_garfo_time = 0.0

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

            if key is not None:
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
                    if subir_garfo():
                        last_garfo = 'p'
                        last_garfo_time = now
                    else:
                        last_garfo = None

                elif key == 'l':
                    if descer_garfo():
                        last_garfo = 'l'
                        last_garfo_time = now
                    else:
                        last_garfo = None

                elif key == ' ':
                    parar_carrinho()
                    parar_garfo()
                    last_mov = None
                    last_garfo = None

                elif key == 'q':
                    break

            if last_mov is not None and (now - last_mov_time) > STOP_TIMEOUT:
                parar_carrinho()
                last_mov = None

            if last_garfo is not None and (now - last_garfo_time) > STOP_TIMEOUT:
                parar_garfo()
                last_garfo = None

            if last_garfo == 'p' and fim_superior_acionado():
                parar_garfo()
                last_garfo = None
                print("Parado: fim superior acionado.")

            if last_garfo == 'l' and fim_inferior_acionado():
                parar_garfo()
                last_garfo = None
                print("Parado: fim inferior acionado.")

            time.sleep(0.01)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        parar_carrinho()
        parar_garfo()
        GPIO.cleanup()

if __name__ == "__main__":
    main()