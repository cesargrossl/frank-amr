import socket
import time
import RPi.GPIO as GPIO

HOST = "127.0.0.1"
PORT = 9000

IN1, IN2 = 17, 27
IN3, IN4 = 22, 23

def gpio_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in (IN1, IN2, IN3, IN4):
        GPIO.setup(p, GPIO.OUT)
        GPIO.output(p, GPIO.LOW)

def parar():  GPIO.output(IN1,0); GPIO.output(IN2,0); GPIO.output(IN3,0); GPIO.output(IN4,0)
def frente(): GPIO.output(IN1,1); GPIO.output(IN2,0); GPIO.output(IN3,1); GPIO.output(IN4,0)
def girar_esq(): GPIO.output(IN1,1); GPIO.output(IN2,0); GPIO.output(IN3,0); GPIO.output(IN4,1)
def girar_dir(): GPIO.output(IN1,0); GPIO.output(IN2,1); GPIO.output(IN3,1); GPIO.output(IN4,0)

def parse_distancias(data):
    setores = {'FRONT': [], 'LEFT': [], 'RIGHT': []}
    for par in data.split(";"):
        if not par: continue
        ang, dist = map(float, par.split(","))
        ang = ang % 360
        if 335 <= ang or ang <= 25:
            setores['FRONT'].append(dist)
        elif 40 <= ang <= 90:
            setores['LEFT'].append(dist)
        elif 270 <= ang <= 320:
            setores['RIGHT'].append(dist)
    return setores

def menor(lista):
    return min(lista) if lista else float("inf")

def main():
    gpio_setup()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        while True:
            data = s.recv(4096).decode()
            setores = parse_distancias(data)
            frente_min = menor(setores['FRONT'])
            if frente_min < 0.30:
                print(f"Obstáculo a {frente_min:.2f}m — desviando...")
                if menor(setores['LEFT']) > menor(setores['RIGHT']):
                    girar_esq()
                else:
                    girar_dir()
                time.sleep(0.35)
                parar()
            else:
                frente()
                time.sleep(0.12)
                parar()
                time.sleep(0.02)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        parar()
        GPIO.cleanup()
