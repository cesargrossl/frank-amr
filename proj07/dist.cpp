import RPi.GPIO as GPIO
import time

# Configura os pinos
TRIG = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def medir_distancia():
    # Gera pulso no TRIG
    GPIO.output(TRIG, False)
    time.sleep(0.000002)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Espera pelo retorno no ECHO
    while GPIO.input(ECHO) == 0:
        inicio = time.time()
    while GPIO.input(ECHO) == 1:
        fim = time.time()

    duracao = fim - inicio
    distancia = (duracao * 34300) / 2  # em cm

    return distancia

try:
    while True:
        dist = medir_distancia()
        if dist >= 20 and dist <= 600:  # alcance do SR04M
            print(f"Distância: {dist:.1f} cm")
        else:
            print("Fora do alcance")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Encerrando...")
    GPIO.cleanup()






sudo apt-get install pigpio
g++ dist.cpp -o dist -lpigpio -lpthread
sudo pigpiod
./dist
