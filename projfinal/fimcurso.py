from gpiozero import Button
from time import sleep

fim_superior = Button(21, pull_up=True)
fim_inferior = Button(20, pull_up=True)

print("Teste dos fins de curso")

while True:

    if fim_superior.is_pressed:
        print("Fim superior: LIVRE")
    else:
        print("Fim superior: ACIONADO")

    if fim_inferior.is_pressed:
        print("Fim inferior: LIVRE")
    else:
        print("Fim inferior: ACIONADO")

    print("-------------------")

    sleep(0.2)