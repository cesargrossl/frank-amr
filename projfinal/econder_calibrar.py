#!/usr/bin/env python3

from gpiozero import DigitalInputDevice
import time

LEFT_GPIO = 16
RIGHT_GPIO = 26

left_encoder = DigitalInputDevice(LEFT_GPIO, pull_up=True, bounce_time=0.001)
right_encoder = DigitalInputDevice(RIGHT_GPIO, pull_up=True, bounce_time=0.001)

left_ticks = 0
right_ticks = 0


def left_pulse():
    global left_ticks
    left_ticks += 1


def right_pulse():
    global right_ticks
    right_ticks += 1


left_encoder.when_activated = left_pulse
right_encoder.when_activated = right_pulse

print("===== TESTE DE ENCODER =====")
print("1) Marque a roda com fita ou caneta")
print("2) Gire EXATAMENTE 1 volta completa")
print("3) Leia o total de pulsos")
print("CTRL+C para sair\n")

last_l = -1
last_r = -1

try:
    while True:
        if left_ticks != last_l or right_ticks != last_r:
            print(f"LEFT={left_ticks} | RIGHT={right_ticks}")
            last_l = left_ticks
            last_r = right_ticks
        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nTeste finalizado")
    print(f"TOTAL FINAL -> LEFT={left_ticks} | RIGHT={right_ticks}")