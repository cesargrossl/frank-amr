#include <iostream>
#include <pigpio.h>
#include <thread>
#include <chrono>
using namespace std;

// Motor (1 motor DC via L298N)
const int IN1 = 17; // GPIO17 -> IN1 L298N
const int IN2 = 27; // GPIO27 -> IN2 L298N

// Fins de curso
const int FIM_FRENTE = 5;  // GPIO5  (pino 29)
const int FIM_TRAS   = 6;  // GPIO6  (pino 31)

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

    // Configura fins de curso como entradas com pull-up
    gpioSetMode(FIM_FRENTE, PI_INPUT);
    gpioSetPullUpDown(FIM_FRENTE, PI_PUD_UP);

    gpioSetMode(FIM_TRAS, PI_INPUT);
    gpioSetPullUpDown(FIM_TRAS, PI_PUD_UP);

    cout << "Controle motor com 2 fins de curso..." << endl;

    bool sentidoFrente = true;

    while (true) {
        if (sentidoFrente) {
            if (gpioRead(FIM_FRENTE) == 0) {
                parar();
                cout << "⚠️ Limite da frente atingido!" << endl;
                sentidoFrente = false; // inverter sentido
                this_thread::sleep_for(chrono::seconds(1));
            } else {
                frente();
            }
        } else {
            if (gpioRead(FIM_TRAS) == 0) {
                parar();
                cout << "⚠️ Limite de trás atingido!" << endl;
                sentidoFrente = true; // inverter sentido
                this_thread::sleep_for(chrono::seconds(1));
            } else {
                tras();
            }
        }

        this_thread::sleep_for(chrono::milliseconds(100));
    }

    gpioTerminate();
}
