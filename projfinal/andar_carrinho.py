#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

# ================== PINOS ==================
IN1 = 17  # Motor A
IN2 = 27
IN3 = 22  # Motor B
IN4 = 23

# ================== GPIO ==================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

def stop():
    GPIO.output(IN1, 0); GPIO.output(IN2, 0)
    GPIO.output(IN3, 0); GPIO.output(IN4, 0)

def forward():
    GPIO.output(IN1, 1); GPIO.output(IN2, 0)
    GPIO.output(IN3, 1); GPIO.output(IN4, 0)

def turn_left():
    GPIO.output(IN1, 0); GPIO.output(IN2, 1)
    GPIO.output(IN3, 1); GPIO.output(IN4, 0)

def turn_right():
    GPIO.output(IN1, 1); GPIO.output(IN2, 0)
    GPIO.output(IN3, 0); GPIO.output(IN4, 1)

try:
    print("Iniciando teste de movimento...")
    while True:

        print("Frente")
        forward()
        time.sleep(2)
        stop()
        time.sleep(1)

        print("Esquerda")
        turn_left()
        time.sleep(2)
        stop()
        time.sleep(1)

        print("Direita")
        turn_right()
        time.sleep(2)
        stop()
        time.sleep(1)

except KeyboardInterrupt:
    print("Parando...")

finally:
    stop()
    GPIO.cleanup()