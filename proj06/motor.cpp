#include <iostream>
#include <pigpio.h>
#include <thread>
#include <chrono>

using namespace std;

// Pinos do motor (entrada L298N)
const int IN1 = 17; // GPIO17 -> IN1
const int IN2 = 27; // GPIO27 -> IN2

void parar() {
    gpioWrite(IN1, 0);
    gpioWrite(IN2, 0);
}

void frente() {
    gpioWrite(IN1, 1);
    gpioWrite(IN2, 0);
}

void tras() {
    gpioWrite(IN1, 0);
    gpioWrite(IN2, 1);
}

int main() {
    if (gpioInitialise() < 0) {
        cerr << "Erro ao inicializar pigpio!" << endl;
        return 1;
    }

    // Configura pinos como saída
    gpioSetMode(IN1, PI_OUTPUT);
    gpioSetMode(IN2, PI_OUTPUT);

    cout << "Teste motor DC simples (frente/tras)" << endl;

    while (true) {
        cout << "Frente..." << endl;
        frente();
        this_thread::sleep_for(chrono::seconds(3));

        cout << "Parar..." << endl;
        parar();
        this_thread::sleep_for(chrono::seconds(1));

        cout << "Tras..." << endl;
        tras();
        this_thread::sleep_for(chrono::seconds(3));

        cout << "Parar..." << endl;
        parar();
        this_thread::sleep_for(chrono::seconds(1));
    }

    gpioTerminate();
    return 0;
}

//g++ -o motor motor.cpp -lpigpio -lrt -lpthread
