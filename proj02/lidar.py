# l298n_pi_lidar.py — Controle com L298N + LIDAR C1 (Slamtec)

import time
import serial
import RPi.GPIO as GPIO
from rplidar import RPLidar
from collections import defaultdict
import math

# ==== Parâmetros de distância (em metros) ====
DIST_THRESH_M = 0.30  # distância de obstáculo frontal (30 cm)
DIST_MARGIN_M = 0.05  # histerese: libera após 35 cm

# ==== Ângulo de compensação (se necessário) ====
ANG_OFFSET_DEG = 0.0

# ==== L298N - Mapeamento dos pinos (BCM) ====
IN1 = 17  # Motor A
IN2 = 27  # Motor A
IN3 = 22  # Motor B
IN4 = 23  # Motor B
T = 3.0   # tempo de movimento

# ==== Setup GPIO ====
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in (IN1, IN2, IN3, IN4):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

def frente():
    print("[AÇÃO] Indo para frente")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def tras():
    print("[AÇÃO] Indo para trás")
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def girar():
    print("[AÇÃO] Girando (obstáculo detectado)")
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def parar():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

# ==== LIDAR - Processamento por setor ====
SECTOR_ANGLES = {
    "FRONT": (-30, 30),
    "LEFT": (60, 120),
    "RIGHT": (-120, -60)
}

def sector_mins_from_stream(lidar, duration_s=0.2):
    end_time = time.time() + duration_s
    dists = defaultdict(list)
    for scan in lidar.iter_scans():
        for (_, angle, distance) in scan:
            angle = (angle + ANG_OFFSET_DEG) % 360
            if distance > 0:
                for name, (a1, a2) in SECTOR_ANGLES.items():
                    a1 = (a1 + 360) % 360
                    a2 = (a2 + 360) % 360
                    in_range = a1 <= angle <= a2 if a1 < a2 else (angle >= a1 or angle <= a2)
                    if in_range:
                        dists[name].append(distance / 1000.0)  # mm → m
        if time.time() > end_time:
            break

    mins = {}
    for name in SECTOR_ANGLES:
        if dists[name]:
            mins[name] = min(dists[name])
        else:
            mins[name] = None
    return mins

# ==== Loop principal ====
def loop_com_lidar(lidar):
    front_blocked = False

    while True:
        mins = sector_mins_from_stream(lidar, duration_s=0.18)
        print("[LIDAR] Leituras (m):", mins)

        front_val = mins.get("FRONT")

        if front_val is not None:
            if not front_blocked and front_val < DIST_THRESH_M:
                print(f"[INFO] Obstáculo detectado à frente ({front_val:.2f} m)")
                front_blocked = True
            elif front_blocked and front_val > DIST_THRESH_M + DIST_MARGIN_M:
                print(f"[INFO] Frente liberada ({front_val:.2f} m)")
                front_blocked = False

        if front_blocked:
            girar()
            time.sleep(0.5)
        else:
            frente()
            time.sleep(0.8)
        parar()
        time.sleep(0.2)

# ==== Programa principal ====
def main():
    setup_gpio()
    lidar = None
    try:
        print("[SETUP] Inicializando LIDAR...")
        lidar = RPLidar('/dev/ttyUSB0')
        lidar.clear_input()
        loop_com_lidar(lidar)
    except KeyboardInterrupt:
        print("\n[EXIT] Encerrando com Ctrl+C.")
    except Exception as e:
        print("[ERRO]", e)
    finally:
        if lidar:
            lidar.stop()
            lidar.disconnect()
        parar()
        GPIO.cleanup()
        print("[FIM] Sistema finalizado.")

if __name__ == "__main__":
    main()
