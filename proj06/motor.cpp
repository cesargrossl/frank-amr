#include <iostream>
#include <pigpio.h>
#include <thread>
#include <chrono>
using namespace std;

// Motor (1 motor DC com 2 fios via L298N)
const int IN1 = 17; // GPIO17 -> IN1 L298N
const int IN2 = 27; // GPIO27 -> IN2 L298N

// Fim de curso
const int FIM = 5; // GPIO5 (pino físico 29)

void parar(){ gpioWrite(IN1,0); gpioWrite(IN2,0); }
void frente(){ gpioWrite(IN1,1); gpioWrite(IN2,0); }
void tras(){ gpioWrite(IN1,0); gpioWrite(IN2,1); }

int main(){
    if (gpioInitialise() < 0) {
        cerr << "Erro ao inicializar pigpio!" << endl;
        return 1;
    }

    // Configura motor
    gpioSetMode(IN1, PI_OUTPUT);
    gpioSetMode(IN2, PI_OUTPUT);

    // Configura fim de curso como entrada com pull-up interno
    gpioSetMode(FIM, PI_INPUT);
    gpioSetPullUpDown(FIM, PI_PUD_UP);

    cout << "Rodando motor, fim de curso para se apertado..." << endl;

    while (true) {
        if (gpioRead(FIM) == 0) { // botão pressionado -> LOW
            parar();
            cout << "⚠️ Fim de curso acionado, motor parado!" << endl;
        } else {
            frente(); // motor roda enquanto não pressionar
        }
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    gpioTerminate();
}
