#include <wiringPi.h>
#include <iostream>
#include <unistd.h> // para sleep em microssegundos

using namespace std;

#define BUZZER 1  // GPIO18 no esquema wiringPi (pino físico 12)

int main() {
    if (wiringPiSetup() == -1) {
        cerr << "Erro ao inicializar wiringPi" << endl;
        return 1;
    }

    pinMode(BUZZER, OUTPUT);

    // Liga buzzer 1 segundo
    digitalWrite(BUZZER, HIGH);
    cout << "Buzzer ligado" << endl;
    delay(1000);

    // Desliga buzzer
    digitalWrite(BUZZER, LOW);
    cout << "Buzzer desligado" << endl;

    return 0;
}
