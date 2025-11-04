# ~/Desktop/raw_probe.py
import serial, sys, time

PORT  = sys.argv[1] if len(sys.argv)>1 else "/dev/ttyUSB0"
BAUD  = int(sys.argv[2]) if len(sys.argv)>2 else 256000

ser = serial.Serial(PORT, BAUD, timeout=2)
ser.reset_input_buffer()
ser.reset_output_buffer()

# Comando GET_INFO: 0xA5 0x50
ser.write(bytes([0xA5, 0x50]))
time.sleep(0.05)
data = ser.read(32)
print("Len:", len(data), "Hex:", data.hex())
ser.close()
