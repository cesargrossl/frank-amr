from gpiozero import Button
from time import sleep

fim_curso = Button(21, pull_up=True)

print("Teste fim de curso no GPIO 20")

while True:
    if fim_curso.is_pressed:
        print("LIVRE")
    else:
        print("PRESSIONADO")
    sleep(0.2)
