#!/usr/bin/env python3

from gpiozero import DigitalInputDevice
from signal import pause
import time

# GPIO usados no seu código
LEFT_GPIO = 16
RIGHT_GPIO = 26

# inicializa sensores
left_encoder = DigitalInputDevice(LEFT_GPIO, pull_up=True, bounce_time=0.001)
right_encoder = DigitalInputDevice(RIGHT_GPIO, pull_up=True, bounce_time=0.001)

left_ticks = 0
right_ticks = 0

def left_pulse():
    global left_ticks
    left_ticks += 1
    print(f"LEFT pulse -> total: {left_ticks}")

def right_pulse():
    global right_ticks
    right_ticks += 1
    print(f"RIGHT pulse -> total: {right_ticks}")

left_encoder.when_activated = left_pulse
right_encoder.when_activated = right_pulse

print("Teste de encoder iniciado")
print("Gire as rodas com a mão")
print("Pressione CTRL+C para sair")

try:
    while True:
        time.sleep(1)
        print(f"TICKS -> LEFT: {left_ticks} | RIGHT: {right_ticks}")

except KeyboardInterrupt:
    print("\nTeste finalizado")