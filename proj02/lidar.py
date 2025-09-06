#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# L298N + RPLIDAR C1 (raw 5B) — desvio reativo com offset, histerese e destravador

import time, sys, serial
import RPi.GPIO as GPIO

# ========= SERIAL =========
PORT = "/dev/ttyUSB0"         # prefira /dev/serial/by-id/...
BAUD = 460800                 # use o que funcionou no seu teste
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
    if hdr != b"\xA5\x5A":
        return None
    rest = ser.read(5)
    if len(rest) != 5: return None
    lm = rest[0] | (rest[1]<<8) | (rest[2]<<16) | (rest[3]<<24)
    data_len = lm & 0x3FFFFFFF
    send_mode = (lm >> 30) & 0x3
    data_type = rest[4]
    return data_len, send_mode, data_type

def decode_measurement_5b(pkt):
    b0,b1,b2,b3,b4 = pkt
    if (b1 & 0x01) == 0:   # check bit
        return None
    start_flag =  b0 & 0x01
    angle_q6   = ((b2 << 7) | (b1 >> 1)) & 0xFFFF
    dist_q2    = (b4 << 8) | b3
    angle_deg  = (angle_q6 / 64.0) % 360.0
    dist_m     = (dist_q2 / 4.0) / 1000.0
    quality    =  b0 >> 2
    return start_flag, angle_deg, dist_m, quality

def lidar_open(port, baud, timeout):
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
# Se o 0° do LIDAR não está apontando para a frente do carrinho, ajuste aqui:
ANG_OFFSET_DEG = 0.0   # teste com 90, 180, -90 se necessário

SECTOR_FRONT = (-25, 25)
SECTOR_LEFT  = (40,  90)
SECTOR_RIGHT = (270, 320)

DIST_THRESH_M   = 0.30
DIST_MARGIN_M   = 0.05   # histerese: libera quando front >= th + margin
MIN_VALID_M     = 0.05
MAX_VALID_M     = 4.00

def _norm(a): 
    a %= 360.0
    return a + 360.0 if a < 0 else a

def _apply_offset(a):
    return _norm(a + ANG_OFFSET_DEG)

def in_sector(angle, sec):
    s,e = map(_norm, sec)
    a = _norm(angle)
    return (s <= e and s <= a <= e) or (s > e and (a >= s or a <= e))

def sector_mins_from_stream(ser, duration_s=0.18):
    t0 = time.time()
    mins = {"FRONT": float("inf"), "LEFT": float("inf"), "RIGHT": float("inf")}
    while time.time() - t0 < duration_s:
        pkt = ser.read(5)
        if len(pkt) != 5: continue
        node = decode_measurement_5b(pkt)
        if not node: continue
        _, ang, dist_m, _ = node
        ang = _apply_offset(ang)
        if not (MIN_VALID_M <= dist_m <= MAX_VALID_M): 
            continue
        if in_sector(ang, SECTOR_FRONT) and dist_m < mins["FRONT"]:
            mins["FRONT"] = dist_m
        if in_sector(ang, SECTOR_LEFT) and dist_m < mins["LEFT"]:
            mins["LEFT"] = dist_m
        if in_sector(ang, SECTOR_RIGHT) and dist_m < mins["RIGHT"]:
            mins["RIGHT"] = dist_m
    for k,v in list(mins.items()):
        mins[k] = (None if v == float("inf") else v)
    return mins

def choose_side(mins):
    L, R = mins.get("LEFT"), mins.get("RIGHT")
    if L is None and R is None: return "esq"   # arbitrário
    if L is None: return "dir"
    if R is None: return "esq"
    return "esq" if L >= R else "dir"

# ========= GPIO / MOVIMENTO =========
IN1, IN2 = 17, 27   # Motor A
IN3, IN4 = 22, 23   # Motor B

FWD_STEP     = 0.12   # s de avanço entre leituras
TURN_TIME    = 0.38   # s de giro no desvio
BACK_TIME    = 0.25   # s de ré no destravador
PAUSE_BRAKE  = 0.08
MAX_TURNS_STUCK = 6   # após N giros seguidos, aplica destravador

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

def main():
    ser = lidar_open(PORT, BAUD, TIMEOUT)
    lidar_start_scan(ser)
    gpio_setup()

    print("Rodando… Ctrl+C para sair.")
    turns_in_row = 0
    front_blocked = False  # para histerese

    try:
        while True:
            mins = sector_mins_from_stream(ser, duration_s=0.18)
            front = mins.get("FRONT")

            # trata None como "livre" (evita giro sem fim quando não há leitura)
            front_val = front if front is not None else 999.0

            # histerese de bloqueio/liberação
            if not front_blocked:
                front_blocked = (front_val <= DIST_THRESH_M)
            else:
                # só libera quando tiver folga suficiente
                front_blocked = not (front_val >= (DIST_THRESH_M + DIST_MARGIN_M))

            if front_blocked:
                # desvio
                motores_parar(); time.sleep(PAUSE_BRAKE)
                lado = choose_side(mins)
                (motores_girar_esq if lado == "esq" else motores_girar_dir)()
                time.sleep(TURN_TIME)
                motores_parar(); time.sleep(PAUSE_BRAKE)
                turns_in_row += 1

                # destravador: se girou várias vezes e não andou, dá ré e muda o lado
                if turns_in_row >= MAX_TURNS_STUCK:
                    motores_tras(); time.sleep(BACK_TIME)
                    motores_parar(); time.sleep(0.05)
                    # gira para o lado oposto ao último escolhido
                    (motores_girar_dir if lado == "esq" else motores_girar_esq)()
                    time.sleep(TURN_TIME * 1.1)
                    motores_parar(); time.sleep(PAUSE_BRAKE)
                    turns_in_row = 0

                # empurra um pouco pra frente depois do giro para sair do “buraco”
                motores_frente(); time.sleep(0.10)
                motores_parar(); time.sleep(0.03)

            else:
                # livre: anda e zera contador de giros
                motores_frente(); time.sleep(FWD_STEP)
                motores_parar(); time.sleep(0.02)
                turns_in_row = 0

    except KeyboardInterrupt:
        pass
    finally:
        try:
            lidar_stop_scan(ser); time.sleep(0.01)
        except Exception:
            pass
        ser.close()
        motores_parar()
        GPIO.cleanup()

if __name__ == "__main__":
    if len(sys.argv) >= 2: PORT = sys.argv[1]
    if len(sys.argv) >= 3: BAUD = int(sys.argv[2])
    main()
