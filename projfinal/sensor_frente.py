import time
import RPi.GPIO as GPIO

TRIG = 25   # pino físico 22
ECHO = 24   # pino físico 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, False)
time.sleep(2)

def medir_distancia():
    # Pulso TRIG de 10us
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Aguarda ECHO subir
    timeout = time.time() + 0.05
    while GPIO.input(ECHO) == 0:
        if time.time() > timeout:
            return None
        inicio = time.time()

    # Aguarda ECHO descer
    while GPIO.input(ECHO) == 1:
        if time.time() > timeout:
            return None
        fim = time.time()

    duracao = fim - inicio
    distancia = (duracao * 34300) / 2
    return distancia

try:
    print("Teste HC-SR04 iniciado (CTRL+C para sair)")
    while True:
        d = medir_distancia()
        if d is None:
            print("Sem leitura")
        else:
            print(f"Distância: {d:.1f} cm")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nEncerrando...")
    GPIO.cleanup()
