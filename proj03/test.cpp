import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 256000

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    # Comando GET_INFO
    ser.write(bytes([0xA5, 0x50]))
    time.sleep(0.1)
    
    data = ser.read(32)
    print(f"Lidos: {len(data)} bytes:", data.hex())
    
    ser.close()
except Exception as e:
    print("Erro:", e)
