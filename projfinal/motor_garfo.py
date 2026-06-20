from gpiozero import OutputDevice, Button
from time import sleep

# Motor - ajuste conforme sua ligação
IN1 = OutputDevice(5)
IN2 = OutputDevice(6)

# Fins de curso
fim_superior = Button(21, pull_up=True)
fim_inferior = Button(20, pull_up=True)

def subir():
    print("Subindo")
    IN1.on()
    IN2.off()

def descer():
    print("Descendo")
    IN1.off()
    IN2.on()

def parar():
    print("Parado")
    IN1.off()
    IN2.off()

print("Controle motor com fim de curso")

# começa subindo
sentido = "subindo"
subir()

while True:

    if sentido == "subindo":
        if not fim_superior.is_pressed:
            print("Fim superior acionado")
            parar()
            sleep(0.5)
            sentido = "descendo"
            descer()

    elif sentido == "descendo":
        if not fim_inferior.is_pressed:
            print("Fim inferior acionado")
            parar()
            sleep(0.5)
            sentido = "subindo"
            subir()

    sleep(0.05)