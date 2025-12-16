#!/usr/bin/env python3
import signal
from evdev import InputDevice, ecodes
import gpiod

# ================= CONFIG =================
EVENT_PATH = "/dev/input/event2"     # CONFIRMADO
CHIP_NAME  = "gpiochip0"             # /dev/gpiochip0

# GPIO BCM offsets (pinos que você já usa)
IN1 = 17  # Motor A
IN2 = 27
IN3 = 22  # Motor B
IN4 = 23

INV_A = True
INV_B = True

DEADZONE = 6000        # ajuste se necessário
TURN_PRIORITY = 1.2    # quanto maior, mais fácil girar

# ================= MOTOR (gpiod v2) =================
class MotorGPIO:
    def __init__(self):
        # pede as 4 linhas como saída e guarda um "request" único
        self.req = gpiod.request_lines(
            CHIP_NAME,
            consumer="motor",
            config={
                IN1: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT, output_value=gpiod.line.Value.INACTIVE),
                IN2: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT, output_value=gpiod.line.Value.INACTIVE),
                IN3: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT, output_value=gpiod.line.Value.INACTIVE),
                IN4: gpiod.LineSettings(direction=gpiod.line.Direction.OUTPUT, output_value=gpiod.line.Value.INACTIVE),
            },
        )
        self.parar()

    def _set_raw(self, a1, a2, b1, b2):
        # set_value espera dict {offset: Value}
        self.req.set_values({
            IN1: gpiod.line.Value.ACTIVE if a1 else gpiod.line.Value.INACTIVE,
            IN2: gpiod.line.Value.ACTIVE if a2 else gpiod.line.Value.INACTIVE,
            IN3: gpiod.line.Value.ACTIVE if b1 else gpiod.line.Value.INACTIVE,
            IN4: gpiod.line.Value.ACTIVE if b2 else gpiod.line.Value.INACTIVE,
        })

    def set(self, a1, a2, b1, b2):
        if INV_A:
            a1, a2 = a2, a1
        if INV_B:
            b1, b2 = b2, b1
        self._set_raw(a1, a2, b1, b2)

    def parar(self):     self.set(0,0,0,0)
    def frente(self):    self.set(1,0,1,0)
    def tras(self):      self.set(0,1,0,1)
    def girar_esq(self): self.set(1,0,0,1)
    def girar_dir(self): self.set(0,1,1,0)

    def close(self):
        try:
            self.parar()
        except:
            pass
        try:
            self.req.release()
        except:
            pass

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
    dev = InputDevice(EVENT_PATH)
    print("Usando:", dev.path, "-", dev.name)

    motor = MotorGPIO()
    running = True

    def stop(*_):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    lx = 0
    ly = 0
    last = None

    print("Analógico esquerdo move | START = STOP | Ctrl+C sai")

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
                    if cmd == "stop":  motor.parar()
                    elif cmd == "fwd":  motor.frente()
                    elif cmd == "rev":  motor.tras()
                    elif cmd == "left": motor.girar_esq()
                    elif cmd == "right":motor.girar_dir()
                    print("CMD:", cmd)

            elif e.type == ecodes.EV_KEY and e.value == 1:
                if e.code == ecodes.BTN_START:
                    motor.parar()
                    last = "stop"
                    print("STOP")

    finally:
        motor.close()

if __name__ == "__main__":
    main()
