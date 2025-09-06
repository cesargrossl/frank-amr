import serial
import time

ser = serial.Serial("/dev/ttyUSB0", 256000)
ser.write(bytes([0xA5, 0x20]))  # comando para iniciar scan
while True:
    data = ser.read(5)
    print(data.hex())
