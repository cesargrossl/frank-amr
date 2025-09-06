#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# L298N + RPLIDAR (C1) — desvio reativo a 30 cm

import time
import math
import sys
import argparse
import RPi.GPIO as GPIO

try:
    from rplidar import RPLidar, RPLidarException
except ImportError:
    print("Instale no venv: pip install rplidar RPi.GPIO")
    sys.exit(1)

# ==== GPIO (BCM) ====
IN1, IN2 = 17, 27   # Motor A
IN3, IN4 = 22, 23   # Motor B

# ==== Movimento ====
FWD_SPEED_TIME = 0.10   # passo para frente entre leituras
TURN_TIME      = 0.35   # tempo de giro para desvio
PAUSE_BRAKE    = 0.08

# ==== Setores LIDAR (graus) ====
SECTOR_FRONT = (-25, 25)
SECTOR_LEFT  = (40, 90)
SECTOR_RIGHT = (270, 320)
DIST_THRESH_M = 0.30
MIN_VALID_MM, MAX_VALID_MM = 50, 4000

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in (IN1, IN2, IN3, IN4):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

def motores_parar():
    GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW); GPIO.output(IN4, GPIO.LOW)

def motores_frente():
    GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)

def motores_girar_esq():
    GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW);  GPIO.output(IN4, GPIO.HIGH)

def motores_girar_dir():
    GPIO.output(IN1, GPIO.LOW);  GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH); GPIO.output(IN4, GPIO.LOW)

def clamp_angle(a):
    a = a % 360.0
    return a + 360.0 if a < 0 else a

def angle_in_sector(angle, sector):
    start, end = map(clamp_angle, sector)
    angle = clamp_angle(angle)
    if start <= end:
        return start <= angle <= end
    return angle >= start or angle <= end

def scan_min_by_sectors(lidar, sectors, max_scans=1, timeout_s=0.5):
    mins_mm = {name: float('inf') for name in sectors}
    t0 = time.time()
    for i, scan in enumerate(lidar.iter_scans(max_buf_meas=500)):
        for (_, ang, dmm) in scan:
            if dmm < MIN_VALID_MM or dmm > MAX_VALID_MM:
                continue
            for name, sec in sectors.items():
                if angle_in_sector(ang, sec) and dmm < mins_mm[name]:
                    mins_mm[name] = dmm
        if i + 1 >= max_scans or (time.time() - t0) > timeout_s:
            break
    return {k: (v/1000.0 if v != float('inf') else None) for k, v in mins_mm.items()}

def escolher_lado_desvio(mins):
    L, R = mins.get('LEFT'), mins.get('RIGHT')
    if L is None and R is None: return 'esq'
    if L is None: return 'dir'
    if R is None: return 'esq'
    return 'esq' if L >= R else 'dir'

def open_lidar(port, baud, timeout=3):
    """Abre o LIDAR. Tenta baud informado; se der erro de descritor, tenta o outro."""
    try_order = [baud] + ([115200] if baud != 115200 else [256000])
    last_exc = None
    for b in try_order:
        try:
            ld = RPLidar(port, baudrate=b, timeout=timeout)
            # tocar o motor e ler um health rápido valida o descritor
            ld.start_motor()
            status, code = ld.get_health()
            # status é string, code é int; se deu certo, usa esse baud
            return ld, b
        except Exception as e:
            last_exc = e
            try:
                # se abriu parcialmente, tente parar e desconectar
                ld.stop()
                ld.stop_motor()
                ld.disconnect()
            except Exception:
                pass
    raise last_exc if last_exc else RPLidarException("Falha ao abrir LIDAR")

def parse_args():
    ap = argparse.ArgumentParser(description="L298N + RPLIDAR (C1) com desvio a 30 cm")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="Porta serial do LIDAR")
    ap.add_argument("--baud", type=int, default=256000, help="Baud do LIDAR (C1=256000)")
    ap.add_argument("--th", type=float, default=DIST_THRESH_M, help="Limite frontal em metros")
    ap.add_argument("--turn", type=float, default=TURN_TIME, help="Tempo de giro (s)")
    ap.add_argument("--step", type=float, default=FWD_SPEED_TIME, help="Passo de avanço (s)")
    return ap.parse_args()

def main():
    args = parse_args()
    global DIST_THRESH_M, TURN_TIME, FWD_SPEED_TIME
    DIST_THRESH_M = args.th
    TURN_TIME = args.turn
    FWD_SPEED_TIME = args.step

    setup_gpio()
    lidar = None
    try:
        lidar, used_baud = open_lidar(args.port, args.baud)
        print(f"LIDAR aberto em {args.port} a {used_baud} baud (C1 esperado = 256000).")
        time.sleep(0.4)

        sectors = {'FRONT': SECTOR_FRONT, 'LEFT': SECTOR_LEFT, 'RIGHT': SECTOR_RIGHT}
        print("Rodando… Ctrl+C para sair.")

        while True:
            mins = scan_min_by_sectors(lidar, sectors, max_scans=1, timeout_s=0.4)
            front = mins.get('FRONT')
            if (front is not None) and (front <= DIST_THRESH_M):
                motores_parar(); time.sleep(PAUSE_BRAKE)
                lado = escolher_lado_desvio(mins)
                (motores_girar_esq if lado == 'esq' else motores_girar_dir)()
                time.sleep(TURN_TIME)
                motores_parar(); time.sleep(PAUSE_BRAKE)
            else:
                motores_frente(); time.sleep(FWD_SPEED_TIME)
                motores_parar();  time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            if lidar is not None:
                try: lidar.stop()
                except Exception: pass
                try: lidar.stop_motor()
                except Exception: pass
                try: lidar.disconnect()
                except Exception: pass
        except Exception:
            pass
        motores_parar()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
