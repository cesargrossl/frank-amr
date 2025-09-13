#include <iostream>
#include <pigpio.h>
#include <unistd.h>

#define TRIG 23
#define ECHO 24

using namespace std;

float medir_distancia() {
    // Pulso no TRIG (10 µs)
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

    // Calcula duração (µs)
    double duracao = fim - inicio;

    // Converte em cm (velocidade do som = 343 m/s)
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
        if (dist >= 2 && dist <= 400) { // alcance útil do HC-SR04
            cout << "Distancia: " << dist << " cm" << endl;
        } else {
            cout << "Fora do alcance" << endl;
        }
        usleep(500000); // 500 ms
    }

    gpioTerminate();
    return 0;
}






sudo apt-get install pigpio
g++ dist.cpp -o dist -lpigpio -lpthread
sudo pigpiod
./dist
