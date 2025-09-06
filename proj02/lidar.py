#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# L298N + RPLIDAR C1 (RAW 5B) — desvio a 30 cm
# Inclui:
#  - calibração automática de ANG_OFFSET_DEG (--calib)
#  - debug de setores (--debug)
#  - cooldown após giro p/ evitar “giro infinito”

import time, sys, math, argparse, serial
import RPi.GPIO as GPIO

# ========= SERIAL =========
DEFAULT_PORT = "/dev/ttyUSB0"     # prefira /dev/serial/by-id/usb-...
DEFAULT_BAUD = 460800             # use o que funcionou no seu teste
TIMEOUT = 1.0

# ========= LIDAR protocolo =========
A5 = 0xA5
CMD = {"STOP":0x25, "RESET":0x40, "SCAN":0x20, "GET_INFO":0x50, "GET_HEALTH":0x52}

def send_cmd(ser, cmd, payload=b""):
    if payload:
        pkt = bytes([A5, cmd, len(payload)]) + payload
        chk = 0
        for b in pkt: chk ^= b
        ser.write(pkt + bytes([chk]))
    else:
        ser.write(bytes([A5, cmd]))

def read_descriptor(ser):
    hdr = ser.read(2)
    if hdr != b"\xA5\x5A": return None
    rest = ser.read(5)
    if len(rest) != 5: return None
    lm = rest[0] | (rest[1]<<8) | (rest[2]<<16) | (rest[3]<<24)
    return (lm & 0x3FFFFFFF), ((lm >> 30) & 0x3), rest[4]

def decode_measurement_5b(pkt):
    b0,b1,b2,b3,b4 = pkt
    if (b1 & 0x01) == 0:   # check bit deve ser 1
        return None
    start_flag =  b0 & 0x01
    angle_q6   = ((b2 << 7) | (b1 >> 1)) & 0xFFFF
    dist_q2    = (b4 << 8) | b3
    angle_deg  = (angle_q6 / 64.0) % 360.0
    dist_m     = (dist_q2 / 4.0) / 1000.0
    quality    =  b0 >> 2
    return start_flag, angle_deg, dist_m, quality

def lidar_open(port, baud, timeout=TIMEOUT):
    ser = serial.Serial(port, baud, timeout=timeout)
    ser.dtr = False; ser.rts = False
    ser.reset_input_buffer(); ser.reset_output_buffer()
    return ser

def lidar_start_scan(ser):
    send_cmd(ser, CMD["SCAN"])
    d = read_descriptor(ser)
    if not d: raise RuntimeError("Sem descriptor após SCAN")

def lidar_stop_scan(ser):
    send_cmd(ser, CMD["STOP"]); time.sleep(0.003)

# ========= SETORES / LÓGICA =========
def _norm(a): 
    a %= 360.0
    return a + 360.0 if a < 0 else a

def in_sector(angle, sector):
    s,e = map(_norm, sector); a = _norm(angle)
    return (s <= e and s <= a <= e) or (s > e and (a >= s or a <= e))

def sector_mins_from_stream(ser, sectors, ang_offset, min_m, max_m, duration_s=0.18):
    t0 = time.time()
    mins = {name: float("inf") for name in sectors}
    while time.time() - t0 < duration_s:
        pkt = ser.read(5)
        if len(pkt) != 5: continue
        node = decode_measurement_5b(pkt)
        if not node: continue
        _, ang, dist_m, _ = node
        ang = _norm(ang + ang_offset)
        if not (min_m <= dist_m <= max_m): continue
        for name, sec in sectors.items():
            if in_sector(ang, sec) and dist_m < mins[name]:
                mins[name] = dist_m
    for k,v in mins.items():
        if v == float("inf"): mins[k] = None
    return mins

def choose_side(mins):
    L, R = mins.get("LEFT"), mins.get("RIGHT")
    if L is None and R is None: return "esq"
    if L is None: return "dir"
    if R is None: return "esq"
    return "esq" if L >= R else "dir"

# ========= GPIO / MOVIMENTO =========
IN1, IN2 = 17, 27   # Motor A
IN3, IN4 = 22, 23   # Motor B

def gpio_setup():
    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
    for p in (IN1,IN2,IN3,IN4):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

def motores_parar():
    GPIO.output(IN1,0); GPIO.output(IN2,0); GPIO.output(IN3,0); GPIO.output(IN4,0)
def motores_frente():
    GPIO.output(IN1,1); GPIO.output(IN2,0); GPIO.output(IN3,1); GPIO.output(IN4,0)
def motores_tras():
    GPIO.output(IN1,0); GPIO.output(IN2,1); GPIO.output(IN3,0); GPIO.output(IN4,1)
def motores_girar_esq():
    GPIO.output(IN1,1); GPIO.output(IN2,0); GPIO.output(IN3,0); GPIO.output(IN4,1)
def motores_girar_dir():
    GPIO.output(IN1,0); GPIO.output(IN2,1); GPIO.output(IN3,1); GPIO.output(IN4,0)

# ========= CALIBRAÇÃO (descobre ANG_OFFSET_DEG) =========
def calibrate_offset(ser, seconds=2.0, min_m=0.05, max_m=6.0):
    """Bina leituras por ângulo e pega a direção com maior distância média como 'frente'."""
    bins = [ [] for _ in range(360) ]
    t0 = time.time()
    while time.time() - t0 < seconds:
        pkt = ser.read(5)
        if len(pkt) != 5: continue
        node = decode_measurement_5b(pkt)
        if not node: continue
        _, ang, d, _ = node
        if min_m <= d <= max_m:
            bins[int(ang) % 360].append(d)
    avgs = [ (sum(b)/len(b) if b else 0.0) for b in bins ]
    best = max(range(360), key=lambda i: avgs[i])
    # Queremos que essa direção (mais livre) vire 0° ⇒ offset = -best
    return (-best) % 360, avgs[best]

# ========= MAIN =========
def main():
    ap = argparse.ArgumentParser(description="L298N + RPLIDAR C1 RAW com desvio e calibração")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--offset", type=float, default=0.0, help="Ângulo (graus) somado às leituras")
    ap.add_argument("--th", type=float, default=0.30, help="Limite frontal (m)")
    ap.add_argument("--margin", type=float, default=0.05, help="Histerese de liberação (m)")
    ap.add_argument("--min", type=float, default=0.05, dest="min_m", help="Filtro min (m)")
    ap.add_argument("--max", type=float, default=4.0,  dest="max_m", help="Filtro max (m)")
    ap.add_argument("--step", type=float, default=0.12, help="Avanço entre leituras (s)")
    ap.add_argument("--turn", type=float, default=0.38, help="Tempo de giro (s)")
    ap.add_argument("--back", type=float, default=0.25, help="Ré do destravador (s)")
    ap.add_argument("--cooldown", type=int, default=5, help="Ciclos ignorando bloqueio após giro")
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--calib", action="store_true", help="Apenas calibra offset e sai")
    args = ap.parse_args()

    # Serial
    ser = lidar_open(args.port, args.baud, TIMEOUT)

    # Calibração (opcional)
    if args.calib:
        print("Calibrando... aponte a FRENTE do carrinho para o espaço mais livre.")
        lidar_start_scan(ser)
        off, dist = calibrate_offset(ser, seconds=2.5)
        lidar_stop_scan(ser); ser.close()
        print(f"Sugestão de ANG_OFFSET_DEG = {off:.1f}°  (dist média máxima ~ {dist:.2f} m)")
        print(f"Rode depois com:  --offset {off:.1f}")
        return

    # Execução normal
    lidar_start_scan(ser)
    gpio_setup()

    sectors = {"FRONT": (-25, 25), "LEFT": (40, 90), "RIGHT": (270, 320)}

    front_blocked = False
    cooldown = 0
    turns_in_row = 0
    MAX_TURNS_STUCK = 6

    try:
        print("Rodando… Ctrl+C para sair.")
        while True:
            mins = sector_mins_from_stream(ser, sectors, args.offset, args.min_m, args.max_m, duration_s=0.20)
            front = mins.get("FRONT")
            front_val = front if front is not None else 999.0

            if args.debug:
                print(f"mins={mins}  blocked={front_blocked}  cooldown={cooldown}")

            # cooldown após giro: ignora bloqueio por N ciclos para "sair do buraco"
            if cooldown > 0:
                cooldown -= 1
                front_blocked = False
            else:
                if not front_blocked:
                    front_blocked = (front_val <= args.th)
                else:
                    front_blocked = not (front_val >= (args.th + args.margin))

            if front_blocked:
                motores_parar(); time.sleep(0.08)
                lado = choose_side(mins)
                (motores_girar_esq if lado == "esq" else motores_girar_dir)()
                time.sleep(args.turn)
                motores_parar(); time.sleep(0.06)
                turns_in_row += 1

                # destravador: muita rotação em seguida → ré + gira pro outro lado
                if turns_in_row >= MAX_TURNS_STUCK:
                    motores_tras(); time.sleep(args.back)
                    motores_parar(); time.sleep(0.05)
                    (motores_girar_dir if lado == "esq" else motores_girar_esq)()
                    time.sleep(args.turn * 1.15)
                    motores_parar(); time.sleep(0.06)
                    turns_in_row = 0

                # empurra um pouco pra frente e ativa cooldown
                motores_frente(); time.sleep(0.12)
                motores_parar(); time.sleep(0.02)
                cooldown = args.cooldown
            else:
                motores_frente(); time.sleep(args.step)
                motores_parar(); time.sleep(0.02)
                turns_in_row = 0

    except KeyboardInterrupt:
        pass
    finally:
        try: lidar_stop_scan(ser)
        except Exception: pass
        ser.close()
        motores_parar()
        GPIO.cleanup()
