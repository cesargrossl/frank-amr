#!/usr/bin/env python3
import serial
import time
import RPi.GPIO as GPIO
from rplidar import RPLidar, RPLidarException

PORT = "/dev/ttyUSB0"
GPIO.setmode(GPIO.BCM)

# === GPIO Motor (ajuste conforme o driver L298N por ex)
MOTOR_A1 = 17
MOTOR_A2 = 27
MOTOR_B1 = 22
MOTOR_B2 = 23

def setup_gpio():
    for pin in [MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2]:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

def frente():
    GPIO.output(MOTOR_A1, GPIO.HIGH)
    GPIO.output(MOTOR_A2, GPIO.LOW)
    GPIO.output(MOTOR_B1, GPIO.HIGH)
    GPIO.output(MOTOR_B2, GPIO.LOW)

def parar():
    GPIO.output(MOTOR_A1, GPIO.LOW)
    GPIO.output(MOTOR_A2, GPIO.LOW)
    GPIO.output(MOTOR_B1, GPIO.LOW)
    GPIO.output(MOTOR_B2, GPIO.LOW)

def direita():
    GPIO.output(MOTOR_A1, GPIO.HIGH)
    GPIO.output(MOTOR_A2, GPIO.LOW)
    GPIO.output(MOTOR_B1, GPIO.LOW)
    GPIO.output(MOTOR_B2, GPIO.HIGH)

def esquerda():
    GPIO.output(MOTOR_A1, GPIO.LOW)
    GPIO.output(MOTOR_A2, GPIO.HIGH)
    GPIO.output(MOTOR_B1, GPIO.HIGH)
    GPIO.output(MOTOR_B2, GPIO.LOW)

def loop_com_lidar(lidar):
    print("[INFO] Iniciando varredura LIDAR...")
    lidar.start_motor()
    lidar.clear_input()

    try:
        for scan in lidar.iter_scans(max_buf_meas=500):
            print("\n[NOVA VOLTA]")
            for (_, angle, distance) in scan:
                print(f"Ângulo: {angle:.2f}°, Distância: {distance:.2f}mm")

                # Lógica simples de movimentação
                if 0 < distance < 500 and (angle > 0 and angle < 60):  # obstáculo à frente
                    print("[AÇÃO] Obstáculo à frente → virar à direita")
                    direita()
                    time.sleep(0.5)
                    parar()
                else:
                    frente()

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[CTRL+C] Parando…")
    except RPLidarException as e:
        print(f"[ERRO] LIDAR falhou: {e}")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        parar()
        GPIO.cleanup()
        print("[FIM] Sistema finalizado.")

def main():
    setup_gpio()
    lidar = None
    try:
        print(f"[SETUP] Conectando no LIDAR via {PORT}...")
        lidar = RPLidar(PORT)
        info = lidar.get_info()
        print(f"[INFO] Modelo: {info['model']}, Firmware: {info['firmware']}, HW: {info['hardware']}, SN: {info['serialnumber']}")
        loop_com_lidar(lidar)
    except RPLidarException as e:
        print(f"[ERRO] LIDAR não pôde ser iniciado: {e}")
    except serial.SerialException as e:
        print(f"[ERRO SERIAL] Verifique a porta {PORT}: {e}")
    finally:
        if lidar:
            lidar.stop()
            lidar.disconnect()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
