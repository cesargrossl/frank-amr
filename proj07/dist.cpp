#include <iostream>
#include <pigpio.h>
#include <chrono>
#include <thread>

using namespace std;

const int TRIG = 23; // GPIO23 (pino físico 16)
const int ECHO = 24; // GPIO24 (pino físico 18)

int main() {
    if (gpioInitialise() < 0) {
        cerr << "Erro ao iniciar pigpio" << endl;
        return 1;
    }

    gpioSetMode(TRIG, PI_OUTPUT);
    gpioSetMode(ECHO, PI_INPUT);

    while (true) {
        // Pulso no TRIG
        gpioWrite(TRIG, 0);
        gpioDelay(2);
        gpioWrite(TRIG, 1);
        gpioDelay(10); // 10 microsegundos
        gpioWrite(TRIG, 0);

        // Espera início do pulso no ECHO
        while (gpioRead(ECHO) == 0);
        uint32_t startTick = gpioTick();

        // Espera fim do pulso
        while (gpioRead(ECHO) == 1);
        uint32_t endTick = gpioTick();

        uint32_t pulseLen = endTick - startTick; // microssegundos

        // Conversão para centímetros (velocidade do som ~343 m/s)
        double distancia = pulseLen / 58.0;

        cout << "Distancia: " << distancia << " cm" << endl;

        this_thread::sleep_for(chrono::milliseconds(500));
    }

    gpioTerminate();
    return 0;
}
