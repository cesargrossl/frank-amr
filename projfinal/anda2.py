import time
import random
import RPi.GPIO as GPIO

# ================== HC-SR04 ==================
TRIG = 25
ECHO = 24

# ================== MOTORES ==================
# Ajuste conforme sua fiação
IN1 = 17  # Motor A
IN2 = 27  # Motor A
IN3 = 22  # Motor B
IN4 = 23  # Motor B

# ================== PARAMETROS ==================
DIST_STOP_CM = 25.0     # se menor que isso, considera obstáculo
DIST_CLEAR_CM = 35.0    # só volta a andar quando passar disso (histerese)
BACK_TIME = 0.35        # tempo de ré (s)
TURN_TIME = 0.45        # tempo girando (s)
LOOP_DT = 0.05          # tempo do loop (s)

# ================== GPIO SETUP ==================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

GPIO.output(TRIG, False)
time.sleep(1)

def stop():
    GPIO.output(IN1, 0); GPIO.output(IN2, 0)
    GPIO.output(IN3, 0); GPIO.output(IN4, 0)

def forward():
    # A e B para frente
    GPIO.output(IN1, 1); GPIO.output(IN2, 0)
    GPIO.output(IN3, 1); GPIO.output(IN4, 0)

def backward():
    GPIO.output(IN1, 0); GPIO.output(IN2, 1)
    GPIO.output(IN3, 0); GPIO.output(IN4, 1)

def turn_left():
    # gira no lugar: A pra trás, B pra frente
    GPIO.output(IN1, 0); GPIO.output(IN2, 1)
    GPIO.output(IN3, 1); GPIO.output(IN4, 0)

def turn_right():
    # gira no lugar: A pra frente, B pra trás
    GPIO.output(IN1, 1); GPIO.output(IN2, 0)
    GPIO.output(IN3, 0); GPIO.output(IN4, 1)

def medir_distancia_cm(timeout_s=0.05):
    # pulso TRIG 10us
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    t_end = time.time() + timeout_s

    # espera subir
    while GPIO.input(ECHO) == 0:
        if time.time() > t_end:
            return None
        start = time.time()

    # espera descer
    while GPIO.input(ECHO) == 1:
        if time.time() > t_end:
            return None
        stop_t = time.time()

    dur = stop_t - start
    return (dur * 34300.0) / 2.0

def dist_filtrada(n=3):
    vals = []
    for _ in range(n):
        d = medir_distancia_cm()
        if d is not None:
            vals.append(d)
        time.sleep(0.01)
    if not vals:
        return None
    vals.sort()
    return vals[len(vals)//2]  # mediana

def main():
    state = "FORWARD"
    last_turn = None

    print("Iniciando desvio com 1 ultrassom (CTRL+C para parar).")
    stop()
    time.sleep(0.5)

    try:
        while True:
            d = dist_filtrada(3)
            if d is None:
                # sem leitura -> pare por segurança
                stop()
                print("Sem leitura -> STOP")
                time.sleep(0.2)
                continue

            print(f"Distância: {d:5.1f} cm | Estado: {state}")

            if state == "FORWARD":
                forward()
                if d < DIST_STOP_CM:
                    stop()
                    state = "BACK"
                    t0 = time.time()

            elif state == "BACK":
                backward()
                if (time.time() - t0) >= BACK_TIME:
                    stop()
                    state = "TURN"
                    t0 = time.time()

                    # alterna esquerda/direita para não ficar preso
                    if last_turn is None:
                        last_turn = random.choice(["L", "R"])
                    else:
                        last_turn = "R" if last_turn == "L" else "L"

            elif state == "TURN":
                if last_turn == "L":
                    turn_left()
                else:
                    turn_right()

                if (time.time() - t0) >= TURN_TIME:
                    stop()
                    # só volta a andar se já “limpou” um pouco
                    if d > DIST_CLEAR_CM:
                        state = "FORWARD"
                    else:
                        # se ainda está perto, faz mais um giro
                        t0 = time.time()

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("\nParando...")
    finally:
        stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
