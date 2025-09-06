#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# l298n_pi_lidar.py — Controle L298N + desvio reativo com LIDAR (RPLIDAR)
#
# Sequência normal: vai em frente, mas se detectar obstáculo <= 0.30 m
# no setor frontal, para e gira para o lado com maior espaço livre.
#
# Requisitos: RPi.GPIO, rplidar
#   sudo apt install -y python3-rpi.gpio
#   pip3 install rplidar
#
# Conexão L298N (BCM):
#   IN1=17, IN2=27  -> Motor A
#   IN3=22, IN4=23  -> Motor B

import time
import math
import sys
import RPi.GPIO as GPIO

try:
    from rplidar import RPLidar
except ImportError:
    print("Instale o pacote do LIDAR:  pip3 install rplidar")
    sys.exit(1)

# ==== Mapeamento dos pinos (BCM) ====
IN1 = 17  # Motor A
IN2 = 27  # Motor A
IN3 = 22  # Motor B
IN4 = 23  # Motor B

# ==== Parâmetros de movimento ====
FWD_SPEED_TIME = 0.10   # passo de avanço (s) entre leituras LIDAR
TURN_TIME      = 0.35   # duração do giro para desviar (s) (ajuste em campo)
PAUSE_BRAKE    = 0.08   # micro pausa após parar (s)

# ==== Parâmetros LIDAR/Setores (graus) ====
# Definição dos setores. 0° = frente, aumenta no sentido anti-horário.
SECTOR_FRONT = (-25, 25)      # frente
SECTOR_LEFT  = (40, 90)       # esquerda (lateral)
SECTOR_RIGHT = (270, 320)     # direita (lateral)
DIST_THRESH_M = 0.30          # 30 cm
MIN_VALID_MM  = 50            # ignora leituras muito curtas
MAX_VALID_MM  = 4000          # ignora muito longe/ruído

# ==== Dispositivo LIDAR ====
LIDAR_PORT = "/dev/ttyUSB0"   # ajuste se necessário

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in (IN1, IN2, IN3, IN4):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

def motores_parar():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

def motores_frente():
    # A: IN1=H, IN2=L | B: IN3=H, IN4=L
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def motores_tras():
    # A: IN1=L, IN2=H | B: IN3=L, IN4=H
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def motores_girar_esq():
    # A frente, B trás -> gira para a esquerda
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def motores_girar_dir():
    # A trás, B frente -> gira para a direita
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def clamp_angle(a):
    """Normaliza ângulo 0..360"""
    a = a % 360.0
    if a < 0:
        a += 360.0
    return a

def angle_in_sector(angle, sector):
    """Testa se um ângulo (0..360) cai no setor (ini,fim) com wrap possível."""
    start, end = sector
    start = clamp_angle(start)
    end   = clamp_angle(end)
    angle = clamp_angle(angle)
    if start <= end:
        return (angle >= start) and (angle <= end)
    else:
        # setor cruza 0°
        return (angle >= start) or (angle <= end)

def scan_min_by_sectors(lidar, sectors, max_scans=1, timeout_s=0.5):
    """
    Coleta até 'max_scans' varreduras e retorna o menor valor (em metros)
    observado em cada setor, ignorando leituras fora de faixa.
    """
    # Inicializa mínimos em +inf
    mins_mm = {name: float('inf') for name in sectors.keys()}
    t0 = time.time()

    for i, scan in enumerate(lidar.iter_scans(max_buf_meas=500)):
        for (_, angle, dist_mm) in scan:
            if dist_mm < MIN_VALID_MM or dist_mm > MAX_VALID_MM:
                continue
            for name, sec in sectors.items():
                if angle_in_sector(angle, sec):
                    if dist_mm < mins_mm[name]:
                        mins_mm[name] = dist_mm
        if i + 1 >= max_scans or (time.time() - t0) > timeout_s:
            break

    # Converte para metros; se ficou inf, retorna None
    mins_m = {name: (val/1000.0 if val != float('inf') else None)
              for name, val in mins_mm.items()}
    return mins_m

def escolher_lado_desvio(mins):
    """
    Decide para qual lado girar (string 'esq' ou 'dir') com base na folga lateral.
    Retorna também as distâncias usadas para debug.
    """
    left = mins.get('LEFT')
    right = mins.get('RIGHT')

    # Se faltar dado de um lado, favorece o outro.
    if left is None and right is None:
        return 'esq'  # arbitrário
    if left is None:
        return 'dir'
    if right is None:
        return 'esq'

    # Maior distância = lado mais livre
    return 'esq' if left >= right else 'dir'

def main():
    setup_gpio()
    lidar = None
    try:
        lidar = RPLidar(LIDAR_PORT)
        lidar.start_motor()
        # Joga fora leituras iniciais (estabilização)
        time.sleep(0.5)

        sectors = {
            'FRONT': SECTOR_FRONT,
            'LEFT' : SECTOR_LEFT,
            'RIGHT': SECTOR_RIGHT,
        }

        print("Rodando… Ctrl+C para sair.")
        while True:
            # Lê setores (1 varredura rápida)
            mins = scan_min_by_sectors(lidar, sectors, max_scans=1, timeout_s=0.4)
            front = mins.get('FRONT')

            # Debug opcional:
            # print(f"FRONT={front:.2f}m  LEFT={mins['LEFT']}  RIGHT={mins['RIGHT']}")

            if (front is not None) and (front <= DIST_THRESH_M):
                # Obstáculo à frente: parar e desviar
                motores_parar()
                time.sleep(PAUSE_BRAKE)

                lado = escolher_lado_desvio(mins)
                if lado == 'esq':
                    motores_girar_esq()
                else:
                    motores_girar_dir()

                # gira por um tempo para abrir caminho
                time.sleep(TURN_TIME)
                motores_parar()
                time.sleep(PAUSE_BRAKE)
            else:
                # Livre: anda em frente por um passo, então reavalia
                motores_frente()
                time.sleep(FWD_SPEED_TIME)
                motores_parar()
                # pequena pausa para estabilidade
                time.sleep(0.02)

    except KeyboardInterrupt:
        pass
    finally:
        try:
            if lidar is not None:
                try:
                    lidar.stop()
                except Exception:
                    pass
                try:
                    lidar.stop_motor()
                except Exception:
                    pass
                try:
                    lidar.disconnect()
                except Exception:
                    pass
        except Exception:
            pass
        motores_parar()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
