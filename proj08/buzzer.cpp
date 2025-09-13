import RPi.GPIO as GPIO
import time

BUZZER = 18  # GPIO18 (pino físico 12)

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER, GPIO.OUT)

print("Buzzer ligado")
GPIO.output(BUZZER, GPIO.HIGH)
time.sleep(1)

print("Buzzer desligado")
GPIO.output(BUZZER, GPIO.LOW)
time.sleep(1)

GPIO.cleanup()
