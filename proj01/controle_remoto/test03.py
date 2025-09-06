import subprocess
import time

# Função para executar comandos gpioset
def gpioset(pin, value):
    cmd = ['gpioset', '--mode=exit', 'gpiochip4', f'{pin}={value}']
    subprocess.run(cmd, check=True)

# Liga IN7 (GPIO11)
print("Ligando IN7 (GPIO11)")
gpioset(11, 0)
time.sleep(2)

# Desliga IN7 e liga IN8 (GPIO13)
print("Desligando IN7, ligando IN8 (GPIO13)")
gpioset(11, 1)
gpioset(13, 0)
time.sleep(2)

# Desliga IN8
print("Desligando IN8")
gpioset(13, 1)
