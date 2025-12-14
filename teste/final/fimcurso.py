from gpiozero import Button
from time import sleep

fim_curso = Button(20, pull_up=True)

print("Teste fim de curso no GPIO 20")

while True:
    if fim_curso.is_pressed:
        print("PRESSIONADO")
    else:
        print("LIVRE")
    sleep(0.2)
