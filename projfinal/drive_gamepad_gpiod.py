#!/usr/bin/env python3
import signal
from evdev import InputDevice, ecodes
import RPi.GPIO as GPIO

# ================= CONFIG =================
EVENT_PATH = "/dev/input/event2"  # seu gamepad

# GPIO BCM
IN1 = 17  # Motor A
IN2 = 27
IN3 = 22  # Motor B
IN4 = 23

INV_A = True
INV_B = True

DEADZONE = 6000
TURN_PRIORITY = 1.2

# ================= MOTOR =================
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in (IN1, IN2, IN3, IN4):
        GPIO.setup(p, GPIO.OUT)
        GPIO.output(p, GPIO.LOW)

def set_raw(a1, a2, b1, b2):
    GPIO.output(IN1, GPIO.HIGH if a1 else GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH if a2 else GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH if b1 else GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH if b2 else GPIO.LOW)

def set_motor(a1, a2, b1, b2):
    if INV_A:
        a1, a2 = a2, a1
    if INV_B:
        b1, b2 = b2, b1
    set_raw(a1, a2, b1, b2)

def parar():     set_motor(0,0,0,0)
def frente():    set_motor(1,0,1,0)
def tras():      set_motor(0,1,0,1)
def girar_esq(): set_motor(1,0,0,1)
def girar_dir(): set_motor(0,1,1,0)

# ================= LOGIC =================
def deadzone(v):
    return 0 if abs(v) < DEADZONE else v

def decide(lx, ly):
    lx = deadzone(lx)
    ly = deadzone(ly)

    if lx == 0 and ly == 0:
        return "stop"

    if abs(lx) > abs(ly) * TURN_PRIORITY:
        return "right" if lx > 0 else "left"

    return "fwd" if ly < 0 else "rev"

# ================= MAIN =================
def main():
    setup_gpio()

    dev = InputDevice(EVENT_PATH)
    print("Usando:", dev.path, "-", dev.name)
    print("Analógico esquerdo move | START = STOP | Ctrl+C sai")

    running = True
    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    lx = 0
    ly = 0
    last = None

    try:
        for e in dev.read_loop():
            if not running:
                break

            if e.type == ecodes.EV_ABS:
                if e.code == ecodes.ABS_X:
                    lx = e.value
                elif e.code == ecodes.ABS_Y:
                    ly = e.value
                else:
                    continue

                cmd = decide(lx, ly)
                if cmd != last:
                    last = cmd
                    if cmd == "stop":
                        parar()
                    elif cmd == "fwd":
                        frente()
                    elif cmd == "rev":
                        tras()
                    elif cmd == "left":
                        girar_esq()
                    elif cmd == "right":
                        girar_dir()
                    print("CMD:", cmd)

            elif e.type == ecodes.EV_KEY and e.value == 1:
                # START = stop imediato
                if e.code == ecodes.BTN_START:
                    parar()
                    last = "stop"
                    print("STOP")

    finally:
        parar()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
