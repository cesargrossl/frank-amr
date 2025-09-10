#include <iostream>
#include <pigpio.h>
#include <thread>
#include <chrono>

using namespace std;

// Definições dos pinos L298N (BCM GPIO)
const int IN1 = 17; // Motor A
const int IN2 = 27; // Motor A
const int IN3 = 22; // Motor B
const int IN4 = 23; // Motor B

// Pinos dos fins de curso
const int LIMITE_ESQ = 5;
const int LIMITE_DIR = 6;

void parar() {
    gpioWrite(IN1, 0);
    gpioWrite(IN2, 0);
    gpioWrite(IN3, 0);
    gpioWrite(IN4, 0);
}

void frente() {
    gpioWrite(IN1, 1);
    gpioWrite(IN2, 0);
    gpioWrite(IN3, 1);
    gpioWrite(IN4, 0);
}

void tras() {
    gpioWrite(IN1, 0);
    gpioWrite(IN2, 1);
    gpioWrite(IN3, 0);
    gpioWrite(IN4, 1);
}

int main() {
    if (gpioInitialise() < 0) {
        cerr << "Erro ao inicializar pigpio" << endl;
        return 1;
    }

    // Configura os pinos do motor como saída
    gpioSetMode(IN1, PI_OUTPUT);
    gpioSetMode(IN2, PI_OUTPUT);
    gpioSetMode(IN3, PI_OUTPUT);
    gpioSetMode(IN4, PI_OUTPUT);

    // Configura os pinos dos fins de curso como entrada com pull-up
    gpioSetMode(LIMITE_ESQ, PI_INPUT);
    gpioSetPullUpDown(LIMITE_ESQ, PI_PUD_UP);

    gpioSetMode(LIMITE_DIR, PI_INPUT);
    g
