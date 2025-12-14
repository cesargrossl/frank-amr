from gpiozero import DigitalInputDevice
from time import sleep

enc1 = DigitalInputDevice(16)
enc2 = DigitalInputDevice(26)

p1 = 0
p2 = 0

def pulso1():
    global p1
    p1 += 1

def pulso2():
    global p2
    p2 += 1

enc1.when_activated = pulso1
enc2.when_activated = pulso2

print("Teste de pulsos iniciado. Gire os encoders...")

while True:
    print(f"Pulsos Encoder 1: {p1} | Pulsos Encoder 2: {p2}")
    sleep(1)
