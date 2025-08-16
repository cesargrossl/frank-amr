import RPi.GPIO as GPIO
import time

# Configurar o modo da GPIO
GPIO.setmode(GPIO.BCM)

# Definir os pinos como saída
rele7 = 11  # IN7
rele8 = 13  # IN8

GPIO.setup(rele7, GPIO.OUT)
GPIO.setup(rele8, GPIO.OUT)

# Desliga todos os relés inicialmente
GPIO.output(rele7, GPIO.HIGH)
GPIO.output(rele8, GPIO.HIGH)

# Liga IN7
print("Ligando IN7")
GPIO.output(rele7, GPIO.LOW)
time.sleep(2)

# Desliga IN7 e liga IN8
print("Desligando IN7, ligando IN8")
GPIO.output(rele7, GPIO.HIGH)
GPIO.output(rele8, GPIO.LOW)
time.sleep(2)

# Desliga IN8
print("Desligando IN8")
GPIO.output(rele8, GPIO.HIGH)

GPIO.cleanup()
