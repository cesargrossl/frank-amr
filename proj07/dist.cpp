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
        gpioDelay(10); // 10 µs
        gpioWrite(TRIG, 0);

        // Espera início do pulso no ECHO (timeout 30ms)
        uint32_t startTick = 0;
        uint32_t timeout = gpioTick() + 30000; // 30.000 µs = 30 ms
        while (gpioRead(ECHO) == 0 && gpioTick() < timeout);
        if (gpioRead(ECHO) == 0) {
            cout << "Distancia: fora de alcance" << endl;
            this_thread::sleep_for(chrono::seconds(2));
            continue;
        }
        startTick = gpioTick();

        // Espera fim do pulso (timeout também)
        timeout = gpioTick() + 30000;
        while (gpioRead(ECHO) == 1 && gpioTick() < timeout);
        if (gpioRead(ECHO) == 1) {
            cout << "Distancia: erro na leitura" << endl;
            this_thread::sleep_for(chrono::seconds(2));
            continue;
        }
        uint32_t endTick = gpioTick();

        uint32_t pulseLen = endTick - startTick; // microssegundos
        double distancia = pulseLen / 58.0;      // cm

        cout << "Distancia: " << distancia << " cm" << endl;

        // mede a cada 2 segundos
        this_thread::sleep_for(chrono::seconds(2));
    }

    gpioTerminate();
    return 0;
}





sudo apt-get install pigpio
g++ dist.cpp -o dist -lpigpio -lpthread
sudo pigpiod
./dist
