#!/usr/bin/env python3
import time
import sys

try:
    import gpiod
except ImportError:
    raise SystemExit("Instale: sudo apt update && sudo apt install -y python3-libgpiod gpiod")

# ================== CONFIGURAÇÕES ==================
CHIP_NAME     = "gpiochip0"  # cheque com `gpioinfo`
IN7           = 17           # BCM 17 (físico 11) -> mapeie p/ "AÇÃO A" (ex.: FRENTE ou ESQ)
IN8           = 27           # BCM 27 (físico 13) -> mapeie p/ "AÇÃO B" (ex.: RÉ ou DIR)
ACTIVE_LOW    = True         # a maioria dos módulos de relé é ativo em LOW

# Mapeamento semântico (troque os nomes conforme seu uso)
ACAO_A_NOME   = "FRENTE"     # o que o IN7 faz no seu controle
ACAO_B_NOME   = "RÉ"         # o que o IN8 faz no seu controle

# Estilo de acionamento:
MODO_PULSO    = True         # True = dá um toque curto; False = manter pressionado enquanto chamar hold_on/hold_off
PULSO_MS      = 250          # duração do pulso em milissegundos
ANTICHATTER_S = 0.2          # tempo mínimo entre comandos para não “treme-ligar”
# ===================================================


# ====== BACKEND gpiod (suporta v2 e v1) ======
def _new_request_v2(chip, pins):
    off = gpiod.LineValue.HIGH if ACTIVE_LOW else gpiod.LineValue.LOW
    off_settings = gpiod.LineSettings(
        direction=gpiod.LineDirection.OUTPUT,
        output_value=off,
        bias=(gpiod.LineBias.PULL_UP if ACTIVE_LOW else gpiod.LineBias.DISABLED)
    )
    req = chip.request_lines(consumer="reles", config={(p,): off_settings for p in pins})
    return req

def _set_value_v2(req, pin, value):
    # value=True->ON, False->OFF
    v_on  = gpiod.LineValue.LOW  if ACTIVE_LOW else gpiod.LineValue.HIGH
    v_off = gpiod.LineValue.HIGH if ACTIVE_LOW else gpiod.LineValue.LOW
    req.set_value(pin, v_on if value else v_off)

def _new_request_v1(chip, pins):
    lines = []
    off_val = 1 if ACTIVE_LOW else 0
    for p in pins:
        line = chip.get_line(p)
        line.request(consumer="reles", type=gpiod.LINE_REQ_DIR_OUT, default_val=off_val)
        lines.append(line)
    return lines  # lista de Line

def _set_value_v1(lines, pin, value):
    on_val  = 0 if ACTIVE_LOW else 1
    off_val = 1 if ACTIVE_LOW else 0
    for ln in lines:
        if ln.offset() == pin:
            ln.set_value(on_val if value else off_val)
            return

class Relays:
    def __init__(self, chip_name, pins):
        self.chip = gpiod.Chip(chip_name)
        self.pins = pins
        self.last_cmd_ts = 0.0
        self.v2 = hasattr(gpiod, "LineSettings")
        self.req = _new_request_v2(self.chip, pins) if self.v2 else _new_request_v1(self.chip, pins)

    def _set(self, pin, on):
        if (time.time() - self.last_cmd_ts) < ANTICHATTER_S and MODO_PULSO:
            # evita múltiplos toques seguidos muito rápidos em modo pulso
            time.sleep(ANTICHATTER_S)
        if self.v2:
            _set_value_v2(self.req, pin, on)
        else:
            _set_value_v1(self.req, pin, on)
        self.last_cmd_ts = time.time()

    def pulse(self, pin, ms=PULSO_MS):
        self._set(pin, True)
        time.sleep(ms/1000.0)
        self._set(pin, False)

    def hold_on(self, pin):
        self._set(pin, True)

    def hold_off(self, pin):
        self._set(pin, False)

    def all_off(self):
        for p in self.pins:
            self._set(p, False)

    def close(self):
        try:
            self.all_off()
        except Exception:
            pass
        if self.v2:
            self.req.release()
        else:
            for ln in self.req:
                try: ln.set_value(1 if ACTIVE_LOW else 0); ln.release()
                except Exception: pass
        self.chip.close()


# ======= ALTO NÍVEL: AÇÕES =======
class MotorControlViaReles:
    def __init__(self, relays, pin_a, pin_b):
        self.relays = relays
        self.pin_a = pin_a
        self.pin_b = pin_b

    # ações SEMPRE EXCLUSIVAS: desliga uma ao ligar a outra
    def _exclusive(self, which_pin, modo_pulso=MODO_PULSO):
        # garante o outro OFF
        other = self.pin_b if which_pin == self.pin_a else self.pin_a
        self.relays.hold_off(other)
        # liga a escolhida
        if modo_pulso:
            self.relays.pulse(which_pin)
        else:
            self.relays.hold_on(which_pin)

    def acao_A(self, modo_pulso=MODO_PULSO):
        # ex.: FRENTE
        self._exclusive(self.pin_a, modo_pulso)

    def acao_B(self, modo_pulso=MODO_PULSO):
        # ex.: RÉ
        self._exclusive(self.pin_b, modo_pulso)

    def parar(self):
        # solta ambos (útil quando está em modo “manter pressionado”)
        self.relays.all_off()


def demo_loop():
    rel = Relays(CHIP_NAME, [IN7, IN8])
    ctl = MotorControlViaReles(rel, IN7, IN8)

    print(f"Pronto. {ACAO_A_NOME}=IN7({IN7})  {ACAO_B_NOME}=IN8({IN8})  "
          f"modo={'PULSO' if MODO_PULSO else 'MANTER'}  (Ctrl+C para sair)")

    try:
        while True:
            # Exemplo: alterna A e B para você observar no controle
            print(f"{ACAO_A_NOME}")
            ctl.acao_A()
            time.sleep(1.5)

            print(f"{ACAO_B_NOME}")
            ctl.acao_B()
            time.sleep(1.5)

            if not MODO_PULSO:
                print("PARAR")
                ctl.parar()
                time.sleep(0.8)

            print("Pausa…")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        rel.close()


if __name__ == "__main__":
    # Se quiser usar via linha de comando:
    #   python3 motores_rele.py A
    #   python3 motores_rele.py B
    #   python3 motores_rele.py STOP
    args = sys.argv[1:]
    rel = Relays(CHIP_NAME, [IN7, IN8])
    ctl = MotorControlViaReles(rel, IN7, IN8)
    try:
        if not args:
            demo_loop()  # demonstração cíclica
        else:
            cmd = args[0].upper()
            if cmd in ("A", "FRENTE", "ESQ", "LEFT", "FORWARD"):
                ctl.acao_A()
            elif cmd in ("B", "RÉ", "RE", "DIR", "RIGHT", "BACK", "BACKWARD"):
                ctl.acao_B()
            elif cmd in ("STOP", "PARAR", "OFF"):
                ctl.parar()
            else:
                print("Uso: A | B | STOP")
    finally:
        rel.close()
