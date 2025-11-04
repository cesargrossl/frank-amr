#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Controle de 'motores' via relés IN7/IN8 (simulando botões do controle).
Compatível com gpiod v2 (LineSettings) e v1 (get_line).
Correções:
 - Tratamento de 'Device or resource busy' na API v1: tenta unexport e re-tenta.
 - Liberação correta das linhas no close().
"""

import time
import sys
import os

try:
    import gpiod
except ImportError:
    raise SystemExit("Instale: sudo apt update && sudo apt install -y python3-libgpiod gpiod")

# ================== CONFIG ==================
CHIP_NAME     = "gpiochip0"   # cheque com `gpioinfo`
IN7           = 17            # BCM 17 (físico 11)
IN8           = 27            # BCM 27 (físico 13)
ACTIVE_LOW    = True          # relé comum: ativo em LOW

ACAO_A_NOME   = "FRENTE"      # mapeie o que IN7 faz
ACAO_B_NOME   = "RÉ"          # mapeie o que IN8 faz

MODO_PULSO    = True          # True: toque curto / False: manter pressionado
PULSO_MS      = 250
ANTICHATTER_S = 0.2           # mínima pausa entre comandos em modo pulso
# ============================================


# ---------- helpers ----------
def _try_sysfs_unexport(pin: int):
    """Tenta liberar pino exportado pelo sysfs (não falha se não conseguir)."""
    base = "/sys/class/gpio"
    try:
        if os.path.exists(f"{base}/gpio{pin}"):
            # requer root; se não tiver, ignoramos o erro
            os.system(f"sh -c 'echo {pin} > {base}/unexport' >/dev/null 2>&1")
            time.sleep(0.05)
    except Exception:
        pass


# ---------- backends gpiod ----------
def _new_request_v2(chip, pins):
    off = gpiod.LineValue.HIGH if ACTIVE_LOW else gpiod.LineValue.LOW
    off_settings = gpiod.LineSettings(
        direction=gpiod.LineDirection.OUTPUT,
        output_value=off,
        bias=(gpiod.LineBias.PULL_UP if ACTIVE_LOW else gpiod.LineBias.DISABLED),
    )
    cfg = {(p,): off_settings for p in pins}
    return chip.request_lines(consumer="reles", config=cfg)

def _set_value_v2(req, pin, on: bool):
    v_on  = gpiod.LineValue.LOW  if ACTIVE_LOW else gpiod.LineValue.HIGH
    v_off = gpiod.LineValue.HIGH if ACTIVE_LOW else gpiod.LineValue.LOW
    req.set_value(pin, v_on if on else v_off)

def _new_request_v1(chip, pins):
    """Cria lines na API v1. Se der busy, tenta unexport e re-tenta 1x."""
    lines = []
    off_val = 1 if ACTIVE_LOW else 0
    for p in pins:
        line = chip.get_line(p)
        try:
            line.request(consumer="reles", type=gpiod.LINE_REQ_DIR_OUT, default_val=off_val)
        except OSError as e:
            # busy -> tentar liberar via sysfs e re-tentar
            if getattr(e, "errno", None) == 16 or "busy" in str(e).lower():
                print(f"[WARN] GPIO {p} ocupado. Tentando liberar via sysfs…")
                _try_sysfs_unexport(p)
                time.sleep(0.05)
                # tenta novamente
                line = chip.get_line(p)
                line.request(consumer="reles", type=gpiod.LINE_REQ_DIR_OUT, default_val=off_val)
            else:
                raise
        lines.append(line)
    return lines

def _set_value_v1(lines, pin, on: bool):
    on_val  = 0 if ACTIVE_LOW else 1
    off_val = 1 if ACTIVE_LOW else 0
    for ln in lines:
        if ln.offset() == pin:
            ln.set_value(on_val if on else off_val)
            return


# ---------- classe Relays ----------
class Relays:
    def __init__(self, chip_name, pins):
        self.chip = gpiod.Chip(chip_name)
        self.pins = pins
        self.last_cmd_ts = 0.0
        self.v2 = hasattr(gpiod, "LineSettings")
        self.req = _new_request_v2(self.chip, pins) if self.v2 else _new_request_v1(self.chip, pins)

    def _set(self, pin, on):
        if MODO_PULSO and (time.time() - self.last_cmd_ts) < ANTICHATTER_S:
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
        # garante tudo desligado e libera recursos
        try:
            self.all_off()
        except Exception:
            pass
        try:
            if self.v2:
                self.req.release()
            else:
                for ln in self.req:
                    try:
                        ln.set_value(1 if ACTIVE_LOW else 0)
                        ln.release()
                    except Exception:
                        pass
        finally:
            self.chip.close()


# ---------- alto nível ----------
class MotorControlViaReles:
    def __init__(self, relays, pin_a, pin_b):
        self.relays = relays
        self.pin_a = pin_a
        self.pin_b = pin_b

    def _exclusive(self, which_pin, modo_pulso=MODO_PULSO):
        other = self.pin_b if which_pin == self.pin_a else self.pin_a
        self.relays.hold_off(other)
        if modo_pulso:
            self.relays.pulse(which_pin)
        else:
            self.relays.hold_on(which_pin)

    def acao_A(self, modo_pulso=MODO_PULSO):
        self._exclusive(self.pin_a, modo_pulso)

    def acao_B(self, modo_pulso=MODO_PULSO):
        self._exclusive(self.pin_b, modo_pulso)

    def parar(self):
        self.relays.all_off()


# ---------- CLI / demo ----------
def demo_loop():
    rel = Relays(CHIP_NAME, [IN7, IN8])
    ctl = MotorControlViaReles(rel, IN7, IN8)
    print(f"Pronto. {ACAO_A_NOME}=IN7({IN7})  {ACAO_B_NOME}=IN8({IN8})  "
          f"modo={'PULSO' if MODO_PULSO else 'MANTER'}  (Ctrl+C para sair)")
    try:
        while True:
            print(ACAO_A_NOME); ctl.acao_A(); time.sleep(1.5)
            print(ACAO_B_NOME); ctl.acao_B(); time.sleep(1.5)
            if not MODO_PULSO:
                print("PARAR"); ctl.parar(); time.sleep(0.8)
            print("Pausa…"); time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        rel.close()

if __name__ == "__main__":
    args = [a.upper() for a in sys.argv[1:]]
    if not args:
        demo_loop()
    else:
        rel = Relays(CHIP_NAME, [IN7, IN8])
        ctl = MotorControlViaReles(rel, IN7, IN8)
        try:
            if args[0] in ("A","FRENTE","ESQ","LEFT","FORWARD"):
                ctl.acao_A()
            elif args[0] in ("B","RÉ","RE","DIR","RIGHT","BACK","BACKWARD"):
                ctl.acao_B()
            elif args[0] in ("STOP","PARAR","OFF"):
                ctl.parar()
            else:
                print("Uso: A | B | STOP")
        finally:
            rel.close()
