#!/usr/bin/env python3
import time, sys, struct, serial
import gpiod
from math import fmod

# ==== CONFIGURAÇÕES ====
PORT = "/dev/ttyUSB0"
BAUD = 460800
TIMEOUT = 1.0

# Setor frontal (ajuste conforme a montagem do LiDAR no robô)
FRONT_CENTER_DEG = 0.0    # defina 0° como a "frente" do robô
FRONT_HALF_WIDTH = 15.0   # setor ±15°
DIST_THRESHOLD_M = 1.0    # obstáculo até 1,0 m
HITS_REQUIRED = 3         # nº mínimo de pontos dentro do setor por volta
PULSE_MS = 300            # tempo do pulso no relé
COOLDOWN_S = 0.7          # antichatter

# GPIO / Relés (assumindo módulos de relé ativos em nível BAIXO)
CHIP_NAME = "gpiochip0"
IN7_GPIO = 17  # pino BCM 17 (físico 11)
IN8_GPIO = 27  # pino BCM 27 (físico 13)
ACTIVE_LOW = True

# ==== PROTOCOLO RPLIDAR (comandos principais) ====
A5 = 0xA5
CMD = {
    "STOP":        0x25,
    "RESET":       0x40,
    "SCAN":        0x20,
    "GET_INFO":    0x50,
    "GET_HEALTH":  0x52,
}

def send_cmd(ser, cmd, payload=b""):
    if payload:
        pkt = bytes([A5, cmd, len(payload)]) + payload
        chk = 0
        for b in pkt:
            chk ^= b
        ser.write(pkt + bytes([chk]))
    else:
        ser.write(bytes([A5, cmd]))

def read_descriptor(ser):
    hdr = ser.read(2)
    if hdr != b"\xA5\x5A":
        return None
    rest = ser.read(5)
    if len(rest) != 5:
        return None
    length_mode = rest[0] | (rest[1]<<8) | (rest[2]<<16) | (rest[3]<<24)
    data_len = length_mode & 0x3FFFFFFF
    send_mode = (length_mode >> 30) & 0x3
    data_type = rest[4]
    return data_len, send_mode, data_type

def decode_node_5b(pkt):
    b0,b1,b2,b3,b4 = pkt
    # b1 bit0 deve ser 1 (check)
    if (b1 & 0x01) == 0:
        return None
    start_flag     =  b0 & 0x01
    quality        =  b0 >> 2
    angle_q6 = ((b2 << 7) | (b1 >> 1)) & 0xFFFF
    dist_q2  = (b4 << 8) | b3
    angle_deg = angle_q6 / 64.0
    dist_m = (dist_q2 / 4.0) / 1000.0
    return start_flag, angle_deg, dist_m, quality

def norm_signed_deg(a):
    """Converte ângulo para faixa [-180, +180)."""
    a = fmod(a + 180.0, 360.0)
    if a < 0: a += 360.0
    return a - 180.0

def in_front_sector(angle_deg, center=FRONT_CENTER_DEG, half=FRONT_HALF_WIDTH):
    # converte ambos p/ signed e verifica se está dentro de ±half
    a = norm_signed_deg(angle_deg - center)
    return (-half) <= a <= (half), a  # retorna também o ângulo relativo

class Relays:
    def __init__(self, chip_name, pins, active_low=True):
        self.chip = gpiod.Chip(chip_name)
        self.active_low = active_low
        cfg = {}
        for p in pins:
            init_val = gpiod.LineValue.HIGH if active_low else gpiod.LineValue.LOW
            cfg[(p,)] = gpiod.LineSettings(
                direction=gpiod.LineDirection.OUTPUT,
                output_value=init_val
            )
        self.req = self.chip.request_lines(consumer="lidar_relays", config=cfg)
        self.pin_to_offset = {p:i for i,p in enumerate(pins)}
        self.pins = pins

    def pulse(self, pin, ms=PULSE_MS):
        idx = self.pins.index(pin)
        on  = gpiod.LineValue.LOW if self.active_low else gpiod.LineValue.HIGH
        off = gpiod.LineValue.HIGH if self.active_low else gpiod.LineValue.LOW
        # liga
        vals = [self.req.get_value(p) for p in self.pins]
        vals[idx] = on
        self.req.set_values(self.pins, vals)
        time.sleep(ms/1000.0)
        # desliga
        vals[idx] = off
        self.req.set_values(self.pins, vals)

def main():
    relays = Relays(CHIP_NAME, [IN7_GPIO, IN8_GPIO], ACTIVE_LOW)
    last_trigger = 0.0

    with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # opcional: consulta saúde
        try:
            send_cmd(ser, CMD["GET_HEALTH"])
            desc = read_descriptor(ser)
            if desc:
                data_len, _, _ = desc
                ser.read(data_len)
        except Exception:
            pass

        # inicia SCAN legado (5 bytes/nó)
        send_cmd(ser, CMD["SCAN"])
        desc = read_descriptor(ser)
        if not desc:
            print("Falha ao receber descriptor do SCAN.", file=sys.stderr)
            return

        # agregadores por volta
        hits_angles_rel = []
        hits_count = 0
        last_start = False
        print("Rodando… Ctrl+C para sair.")

        try:
            while True:
                pkt = ser.read(5)
                if len(pkt) != 5:
                    continue
                node = decode_node_5b(pkt)
                if not node:
                    continue
                start, ang_abs, dist_m, q = node

                # detecta início de nova volta
                if start and not last_start:
                    # fim da volta anterior → decide se aciona
                    now = time.time()
                    if hits_count >= HITS_REQUIRED and (now - last_trigger) > COOLDOWN_S:
                        # média dos ângulos relativos dos hits (lado do obstáculo)
                        mean_rel = sum(hits_angles_rel)/len(hits_angles_rel)
                        if mean_rel < 0:
                            # obstáculo mais à esquerda → aciona IN7
                            print(f"[TRIGGER] obstáculo à ESQUERDA (avg {mean_rel:.1f}°) → IN7")
                            relays.pulse(IN7_GPIO)
                        else:
                            # obstáculo mais à direita → aciona IN8
                            print(f"[TRIGGER] obstáculo à DIREITA (avg {mean_rel:.1f}°) → IN8")
                            relays.pulse(IN8_GPIO)
                        last_trigger = now

                    # reseta para a próxima volta
                    hits_count = 0
                    hits_angles_rel.clear()

                last_start = bool(start)

                # filtra por setor frontal e distância
                in_front, rel_deg = in_front_sector(ang_abs)
                if in_front and 0.05 <= dist_m <= DIST_THRESHOLD_M:
                    hits_count += 1
                    hits_angles_rel.append(rel_deg)

        except KeyboardInterrupt:
            pass
        finally:
            # STOP
            send_cmd(ser, CMD["STOP"])
            time.sleep(0.01)

if __name__ == "__main__":
    main()
