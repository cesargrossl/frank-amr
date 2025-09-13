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

    // Espera início do pulso no ECHO (timeout máx ~38ms para 6m)
    double inicio, fim;
    int timeout = 0;
    while (gpioRead(ECHO) == 0) {
        inicio = gpioTick();
        if (++timeout > 40000) return -1; // sem resposta
    }

    timeout = 0;
    while (gpioRead(ECHO) == 1) {
        fim = gpioTick();
        if (++timeout > 40000) return -1; // sem resposta
    }

    double duracao = fim - inicio; // µs
    float distancia = (duracao * 0.0343) / 2; // cm

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

        if (dist < 0) {
            cout << "Sem leitura" << endl; // objeto muito longe ou sem eco
        }
        else if (dist < 20) {
            cout << "Muito perto (<20 cm)" << endl;
        }
        else {
            cout << "Distancia: " << dist << " cm" << endl;
        }

        usleep(500000); // 500 ms
    }

    gpioTerminate();
    return 0;
}
