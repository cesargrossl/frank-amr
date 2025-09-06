#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# L298N + RPLIDAR C1 (protocolo raw 5B) — desvio reativo a 30 cm

import time, sys, math, serial
import RPi.GPIO as GPIO

# ========= SERIAL =========
PORT = "/dev/ttyUSB0"   # prefira: /dev/serial/by-id/usb-...
BAUD = 460800           # C1: use o que funcionou no seu teste (você confirmou 460800)
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
    # bit0 de b1 deve ser 1 (check bit)
    if (b1 & 0x01) == 0:
        return None
    start_flag     =  b0 & 0x01
    angle_q6       = ((b2 << 7) | (b1 >> 1)) & 0xFFFF
    dist_q2        = (b4 << 8) | b3
    angle_deg      = (angle_q6 / 64.0) % 360.0
    dist_m         = (dist_q2 / 4.0) / 1000.0
    quality        =  b0 >> 2
    return start_flag, angle_deg, dist_m, quality

def lidar_open(port=PORT, baud=BAUD, timeout=TIMEOUT):
    ser = serial.Serial(port, baud, timeout=timeout)
    ser.dtr = False; ser.rts = False
    ser.reset_input_buffer(); ser.reset_output_buffer()
    return ser

def lidar_info(ser):
    send_cmd(ser, CMD["GET_INFO"])
    d = read_descriptor(ser)
    if not d: return None
    n,_,_ = d
    data = ser.read(n)
    if len(data) != n or n != 20: return None
    model, fw_minor, fw_major, hw = data[0], data[1], data[2], data[3]
    serial_hex = data[4:20][::-1].hex()
    return dict(model=model, fw=f"{fw_major}.{fw_minor}", hw=hw, serial=serial_hex)

def lidar_health(ser):
    send_cmd(ser, CMD["GET_HEALTH"])
    d = read_descriptor(ser)
    if not d: return None
    n,_,_ = d
    data = ser.read(n)
    if len(data) != n or n != 3: return None
    st = data[0]; err = data[1] | (data[2]<<8)
    return dict(status=st, errcode=err)

def lidar_start_scan(ser):
    send_cmd(ser, CMD["SCAN"])
    d = read_descriptor(ser)
    if not d: raise RuntimeError("Sem descriptor após SCAN")
    n,_,dtype = d
    if n != 5 or dtype != 0x81:
        print(f"[WARN] Descriptor inesperado: len={n} type=0x{dtype:02X} (prosseguindo)")

def lidar_stop_scan(ser):
    send_cmd(ser, CMD["STOP"]); time.sleep(0.002)

# ========= SETORES / LÓGICA =========
SECTOR_FRONT = (-25, 25)
SECTOR_LEFT  = (40,  90)
SECTOR_RIGHT = (270, 320)
DIST_THRESH_M = 0.30
MIN_VALID_M   = 0.05
MAX_VALID_M   = 4.00

def _norm(a): 
    a %= 360.0
    return a + 360.0 if a < 0 else a

def in_sector(angle, sec):
    s,e = map(_norm, sec)
    a = _norm(angle)
    return (s <= e and s <= a <= e) or (s > e and (a >= s or a <= e))

def sector_mins_from_stream(ser, duration_s=0.15):
    """Lê pacotes por ~duration_s e retorna menor distância por setor (m)."""
    t0 = time.time()
    mins = {"FRONT": float("inf"), "LEFT": float("inf"), "RIGHT": float("inf")}
    while time.time() - t0 < duration_s:
        pkt = ser.read(5)
        if len(pkt) != 5: continue
        node = decode_measurement_5b(pkt)
        if not node: continue
        _, ang, dist_m, q = node
        if not (MIN_VALID_M <= dist_m <= MAX_VALID_M): 
            continue
        if in_sector(ang, SECTOR_FRONT) and dist_m < mins["FRONT"]:
            mins["FRONT"] = dist_m
        if in_sector(ang, SECTOR_LEFT) and dist_m < mins["LEFT"]:
            mins["LEFT"] = dist_m
        if in_sector(ang, SECTOR_RIGHT) and dist_m < mins["RIGHT"]:
            mins["RIGHT"] = dist_m
    # None se não houve leitura
    for k,v in list(mins.items()):
        mins[k] = (None if v == float("inf") else v)
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

FWD_STEP   = 0.10   # s de avanço entre leituras
TURN_TIME  = 0.35   # s de giro no desvio
PAUSE_BRAK = 0.08

def gpio_setup():
    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
    for p in (IN1,IN2,IN3,IN4):
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)

def motores_parar():
    GPIO.output(IN1,0); GPIO.output(IN2,0); GPIO.output(IN3,0); GPIO.output(IN4,0)

def motores_frente():
    GPIO.output(IN1,1); GPIO.output(IN2,0); GPIO.output(IN3,1); GPIO.output(IN4,0)

def motores_girar_esq():
    GPIO.output(IN1,1); GPIO.output(IN2,0); GPIO.output(IN3,0); GPIO.output(IN4,1)

def motores_girar_dir():
    GPIO.output(IN1,0); GPIO.output(IN2,1); GPIO.output(IN3,1); GPIO.output(IN4,0)

# ========= MAIN LOOP =========
def main():
    print(f"Abrindo LIDAR em {PORT} @ {BAUD}…")
    ser = lidar_open(PORT, BAUD, TIMEOUT)
    try:
        inf = lidar_info(ser)
        if inf: print(f"INFO: model={inf['model']} fw={inf['fw']} hw={inf['hw']} sn={inf['serial']}")
        hl = lidar_health(ser)
        if hl:  print(f"HEALTH: status={hl['status']} err=0x{hl['errcode']:04X}")
        lidar_start_scan(ser)

        gpio_setup()
        print("Rodando… Ctrl+C para sair.")
        while True:
            mins = sector_mins_from_stream(ser, duration_s=0.15)
            front = mins.get("FRONT")
            if (front is not None) and (front <= DIST_THRESH_M):
                motores_parar(); time.sleep(PAUSE_BRAK)
                lado = choose_side(mins)
                (motores_girar_esq if lado == "esq" else motores_girar_dir)()
                time.sleep(TURN_TIME)
                motores_parar(); time.sleep(PAUSE_BRAK)
            else:
                motores_frente(); time.sleep(FWD_STEP)
                motores_parar();  time.sleep(0.02)
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
    # Permite passar porta/baud por argumentos (opcional)
    if len(sys.argv) >= 2: PORT = sys.argv[1]
    if len(sys.argv) >= 3: BAUD = int(sys.argv[2])
    main()
