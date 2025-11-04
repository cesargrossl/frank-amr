#include <iostream>
#include <pigpio.h>
#include <unistd.h>

#define TRIG 23  // GPIO23 (pino físico 16)
#define ECHO 24  // GPIO24 (pino físico 18)

using namespace std;

float medir_distancia() {
    // Pulso no TRIG
    gpioWrite(TRIG, 0);
    gpioDelay(2);
    gpioWrite(TRIG, 1);
    gpioDelay(10);
    gpioWrite(TRIG, 0);

    // Espera início do pulso no ECHO
    while (gpioRead(ECHO) == 0);
    double inicio = gpioTick();

    // Espera fim do pulso no ECHO
    while (gpioRead(ECHO) == 1);
    double fim = gpioTick();

    // Duração em microssegundos
    double duracao = fim - inicio;

    // Distância em cm
    float distancia = (duracao * 0.0343) / 2;

    return distancia;
}

int main() {
    if (gpioInitialise() < 0) {
        cerr << "Erro ao iniciar pigpio" << endl;
        return 1;
    }

    gpioSetMode(TRIG, PI_OUTPUT);
    gpioSetMode(ECHO, PI_INPUT);

    while (true) {
        float dist = medir_distancia();
        if (dist >= 20 && dist <= 600) { // alcance do JSN-SR04M
            cout << "Distancia: " << dist << " cm" << endl;
        } else {
            cout << "Fora do alcance" << endl;
        }
        usleep(500000); // 500 ms
    }

    gpioTerminate();
    return 0;
}
