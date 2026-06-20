from gpiozero import OutputDevice
from time import sleep

gpio_teste = [
    2,3,4,5,6,7,
    8,9,10,11,12,13,
    14,15,16,17,18,19,
    20,21,22,23,24,25,26,27
]

for gpio in gpio_teste:

    print(f"Testando GPIO {gpio}")

    try:
        pino = OutputDevice(gpio)

        pino.on()
        sleep(1)

        pino.off()
        sleep(1)

        pino.close()

    except Exception as e:
        print(e)