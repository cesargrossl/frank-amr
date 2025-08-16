import RPi.GPIO as GPIO
import time

# Define os GPIOs usados
RELE7 = 17  # IN7 (pino físico 11)
RELE8 = 27  # IN8 (pino físico 13)

# Configura os pinos
GPIO.setmode(GPIO.BCM)  # usa numeração GPIO
GPIO.setup(RELE7, GPIO.OUT)
GPIO.setup(RELE8, GPIO.OUT)

# Inicializa relés desligados (nível alto)
GPIO.output(RELE7, True)
GPIO.output(RELE8, True)

try:
    while True:
        print("Acionando RELE7")
        GPIO.output(RELE7, False)  # Liga relé 7
        time.sleep(0.5)
        GPIO.output(RELE7, True)   # Desliga relé 7
        time.sleep(2)

        print("Acionando RELE8")
        GPIO.output(RELE8, False)  # Liga relé 8
        time.sleep(0.5)
        GPIO.output(RELE8, True)   # Desliga relé 8
        time.sleep(2)

except KeyboardInterrupt:
    print("Encerrando...")
    GPIO.cleanup()
