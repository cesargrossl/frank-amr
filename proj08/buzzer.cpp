#include <iostream>   // cout, endl
#include <unistd.h>   // sleep()
#include <pigpio.h>   // funções da pigpio

using namespace std;

#define BUZZER 18  // GPIO18 (pino físico 12)

// Função: buzina contínua
void buzina(int tempoSegundos) {
    gpioWrite(BUZZER, 1);
    sleep(tempoSegundos);
    gpioWrite(BUZZER, 0);
}


// Função: bip de ré (liga/desliga em loop)
void beepRe(int repeticoes, int tempoLigadoMs, int tempoDesligadoMs) {
    for (int i = 0; i < repeticoes; i++) {
        gpioWrite(BUZZER, 1);
        gpioDelay(tempoLigadoMs * 1000);   // gpioDelay usa microssegundos
        gpioWrite(BUZZER, 0);
        gpioDelay(tempoDesligadoMs * 1000);
    }
}

int main() {
    if (gpioInitialise() < 0) {
        cerr << "Erro ao inicializar pigpio!" << endl;
        return 1;
    }

    gpioSetMode(BUZZER, PI_OUTPUT);

    cout << ">>> Teste: buzina contínua" << endl;
    buzina(2);   // buzina 2 segundos

    sleep(1);

    cout << ">>> Teste: som de ré (bip-bip)" << endl;
    beepRe(10, 300, 300); // 10 repetições, 300ms ligado e 300ms desligado

    gpioTerminate();
    return 0;
}

g++ buzzer.cpp -o buzzer -lpigpio -lrt -pthread