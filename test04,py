#!/usr/bin/env python3
import time

try:
    import gpiod
except ImportError:
    raise SystemExit("Instale: sudo apt update && sudo apt install python3-libgpiod gpiod")

CHIP_NAME = "gpiochip0"   # confira com `gpioinfo` (no Pi costuma ser gpiochip0)
IN7 = 17                  # pino físico 11
IN8 = 27                  # pino físico 13
ACTIVE_LOW = True         # maioria dos módulos de relé é ativo em LOW

def main_v2():
    chip = gpiod.Chip(CHIP_NAME)
    # Mantém “desligado” firme (HIGH quando ACTIVE_LOW=True)
    off_settings = gpiod.LineSettings(
        direction=gpiod.LineDirection.OUTPUT,
        output_value=(gpiod.LineValue.HIGH if ACTIVE_LOW else gpiod.LineValue.LOW),
        bias=(gpiod.LineBias.PULL_UP if ACTIVE_LOW else gpiod.LineBias.DISABLED)
    )
    req = chip.request_lines(
        consumer="reles",
        config={(IN7,): off_settings, (IN8,): off_settings}
    )

    def relay_on(pin):
        req.set_value(pin, gpiod.LineValue.LOW if ACTIVE_LOW else gpiod.LineValue.HIGH)

    def relay_off(pin):
        req.set_value(pin, gpiod.LineValue.HIGH if ACTIVE_LOW else gpiod.LineValue.LOW)

    try:
        print("Loop iniciado. Ctrl+C para sair.")
        while True:
            print("IN7 ON por 2s")
            relay_on(IN7)
            relay_off(IN8)
            time.sleep(2)

            print("IN7 OFF, IN8 ON por 2s")
            relay_off(IN7)
            relay_on(IN8)
            time.sleep(2)

            print("IN8 OFF, aguardando 6s")
            relay_off(IN8)
            time.sleep(6)
    except KeyboardInterrupt:
        pass
    finally:
        # garante tudo desligado
        try:
            relay_off(IN7); relay_off(IN8)
        except Exception:
            pass
        req.release()
        chip.close()

def main_v1():
    chip = gpiod.Chip(CHIP_NAME)
    line7 = chip.get_line(IN7)
    line8 = chip.get_line(IN8)

    off_val = 1 if ACTIVE_LOW else 0   # “desligado”
    on_val  = 0 if ACTIVE_LOW else 1   # “ligado”

    # OBS: na API v1 é default_val (singular)
    line7.request(consumer="reles", type=gpiod.LINE_REQ_DIR_OUT, default_val=off_val)
    line8.request(consumer="reles", type=gpiod.LINE_REQ_DIR_OUT, default_val=off_val)

    try:
        print("Loop iniciado. Ctrl+C para sair.")
        while True:
            print("IN7 ON por 2s")
            line7.set_value(on_val)
            line8.set_value(off_val)
            time.sleep(2)

            print("IN7 OFF, IN8 ON por 2s")
            line7.set_value(off_val)
            line8.set_value(on_val)
            time.sleep(2)

            print("IN8 OFF, aguardando 6s")
            line8.set_value(off_val)
            time.sleep(6)
    except KeyboardInterrupt:
        pass
    finally:
        line7.set_value(off_val)
        line8.set_value(off_val)
        line7.release()
        line8.release()
        chip.close()

if __name__ == "__main__":
    # Detecta API: se existir LineSettings -> v2; senão v1
    if hasattr(gpiod, "LineSettings"):
        main_v2()
    else:
        main_v1()
