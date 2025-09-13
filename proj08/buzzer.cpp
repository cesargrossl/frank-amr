#include <pigpio.h>
#include <iostream>
#include <unistd.h> // sleep

using namespace std;

#define BUZZER 18  // GPIO18 (pino físico 12)

int main() {
    if (gpioInitialise() < 0) {
        cerr << "Erro ao inicializar pigpio!" << endl;
        return 1;
    }

    gpioSetMode(BUZZER, PI_OUTPUT);

    cout << "Buzzer ligado" << endl;
    gpioWrite(BUZZER, 1);
    sleep(1);

    cout << "Buzzer desligado" << endl;
    gpioWrite(BUZZER, 0);
    sleep(1);

    gpioTerminate();
    return 0;
}
g++ buzzer.cpp -o buzzer -lpigpio -lrt -pthread