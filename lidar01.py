# (opcional) dar permissão à porta serial sem sudo
# sudo usermod -a -G dialout $USER
# faça logout/login para valer o grupo

# dependência Python
# sudo apt update
# sudo apt install -y python3-pip
# pip3 install pyserial

#!/usr/bin/env python3
import serial, struct, sys, time

PORT = "/dev/ttyUSB0"     # ajuste se necessário
BAUD = 460800             # C1
TIMEOUT = 1.0

# ===== Helpers: protocolo =====
A5 = 0xA5
CMD = {
    "STOP":        0x25,
    "RESET":       0x40,
    "SCAN":        0x20,   # legacy scan (dados: 5 bytes por amostra)
    "FORCE_SCAN":  0x21,
    "GET_INFO":    0x50,
    "GET_HEALTH":  0x52,
}

def send_cmd(ser, cmd, payload=b""):
    """Envia um comando no formato: A5 <cmd> [len payload] [payload] [checksum]"""
    if payload:
        pkt = bytes([A5, cmd, len(payload)]) + payload
        chk = 0
        for b in pkt:
            chk ^= b
        ser.write(pkt + bytes([chk]))
    else:
        ser.write(bytes([A5, cmd]))

def read_descriptor(ser):
    """Lê o descriptor: A5 5A <len(30b)+mode(2b)> <datatype>"""
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

def decode_measurement_5b(pkt):
    """
    Decodifica um nó de 5 bytes (legacy SCAN).
    Formato (v2.x):
      b0: S(1) | ~S(1) | quality(6)
      b1: check bit (1) | angle_q6 low(7)
      b2: angle_q6 high(8)
      b3: distance_q2 low(8)
      b4: distance_q2 high(8)
    angle = angle_q6 / 64.0   (graus)
    distance_mm = distance_q2 / 4.0
    """
    b0,b1,b2,b3,b4 = pkt
    start_flag     =  b0 & 0x01
    not_start_flag = (b0 >> 1) & 0x01
    quality        =  b0 >> 2
    # bit0 de b1 é sempre 1 (check bit)
    if (b1 & 0x01) == 0:
        return None
    angle_q6 = ((b2 << 7) | (b1 >> 1)) & 0xFFFF
    dist_q2  = (b4 << 8) | b3
    angle_deg = angle_q6 / 64.0
    dist_m = (dist_q2 / 4.0) / 1000.0
    return start_flag, angle_deg, dist_m, quality

def get_info(ser):
    send_cmd(ser, CMD["GET_INFO"])
    desc = read_descriptor(ser)
    if not desc: return None
    data_len, mode, dtype = desc
    data = ser.read(data_len)
    if len(data) != data_len:
        return None
    # Estrutura de 20 bytes: model, fw minor, fw major, hw, serial[16]
    if data_len == 20:
        model, fw_minor, fw_major, hw = data[0], data[1], data[2], data[3]
        serial_hex = data[4:20][::-1].hex()  # LSB first na doc
        return dict(model=model, fw=f"{fw_major}.{fw_minor}", hw=hw, serial=serial_hex)
    return None

def get_health(ser):
    send_cmd(ser, CMD["GET_HEALTH"])
    desc = read_descriptor(ser)
    if not desc: return None
    data_len, mode, dtype = desc
    data = ser.read(data_len)
    if len(data) != data_len or data_len != 3:
        return None
    status = data[0]  # 0:Good, 1:Warning, 2:Error
    errcode = data[1] | (data[2]<<8)
    return dict(status=status, errcode=errcode)

def start_scan(ser):
    send_cmd(ser, CMD["SCAN"])
    desc = read_descriptor(ser)
    if not desc:
        raise RuntimeError("Sem descriptor após SCAN")
    data_len, mode, dtype = desc
    if data_len != 5 or dtype != 0x81:
        # Ainda é válido receber 0x81 com 5 bytes no SCAN; se vier outro modo, tratar diferente
        print(f"Aviso: tipo={hex(dtype)} len={data_len}, esperando 5/0x81 (legacy).")
    return True

def stop_scan(ser):
    send_cmd(ser, CMD["STOP"])
    time.sleep(0.002)  # ≥1 ms

def main():
    print(f"Abrindo {PORT} @ {BAUD}…")
    with serial.Serial(PORT, BAUD, timeout=TIMEOUT) as ser:
        # Flush inicial
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # Info
        info = get_info(ser)
        if info:
            print(f"Modelo={info['model']}  FW={info['fw']}  HW={info['hw']}  SN={info['serial']}")
        else:
            print("Não foi possível ler GET_INFO (seguindo).")

        # Health
        h = get_health(ser)
        if h:
            st = ["GOOD","WARNING","ERROR"][h['status']] if h['status']<=2 else str(h['status'])
            print(f"Health: {st}  err=0x{h['errcode']:04X}")
            if h['status'] == 2:
                print("Sensor em Protection Stop; tentando RESET…")
                send_cmd(ser, CMD["RESET"]); time.sleep(0.01)
        else:
            print("Não foi possível ler GET_HEALTH (seguindo).")

        # Inicia varredura
        start_scan(ser)
        print("Lendo amostras (Ctrl+C para sair)…")
        scan_id = 0
        last_start = False
        try:
            while True:
                pkt = ser.read(5)
                if len(pkt) != 5:
                    continue
                node = decode_measurement_5b(pkt)
                if not node:
                    continue
                start, ang, dist_m, qual = node
                if start and not last_start:
                    scan_id += 1
                    print(f"\n--- nova volta #{scan_id} ---")
                last_start = bool(start)
                # imprime apenas alguns pontos por segundo para não lotar o terminal
                if int(ang) % 30 == 0:
                    print(f"θ={ang:7.2f}°   d={dist_m:6.3f} m   q={qual}")
        except KeyboardInterrupt:
            print("\nParando…")
        finally:
            stop_scan(ser)
            time.sleep(0.01)

if __name__ == "__main__":
    try:
        main()
    except serial.SerialException as e:
        print("Erro serial:", e, file=sys.stderr)
        sys.exit(1)
