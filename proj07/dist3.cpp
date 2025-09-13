#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>  // Para sleep

#define TRIG 16  // GPIO 16 como trigger (saída)
#define ECHO 18  // GPIO 18 como echo (entrada)
#define MIN_DISTANCE 20  // Ignorar abaixo de 20 cm
#define MAX_DISTANCE 400 // Ignorar acima de 400 cm

static int ping() {
    long ping_time = 0;
    long pong_time = 0;
    float distance = 0;
    long timeout = 500000;  // Timeout de 0.5s (~170m max)

    // Envia pulso trigger
    digitalWrite(TRIG, LOW);
    usleep(2);  // Pequeno delay
    digitalWrite(TRIG, HIGH);
    usleep(10); // Pulso de 10us
    digitalWrite(TRIG, LOW);

    // Espera echo começar (HIGH)
    long start = micros();
    while (digitalRead(ECHO) == LOW && (micros() - start < timeout)) {
        // Loop até HIGH ou timeout
    }
    if (digitalRead(ECHO) == LOW) {
        return 0;  // Timeout no start
    }
    ping_time = micros();

    // Espera echo acabar (LOW)
    start = micros();
    while (digitalRead(ECHO) == HIGH && (micros() - start < timeout)) {
        // Loop até LOW ou timeout
    }
    if (digitalRead(ECHO) == HIGH) {
        return 0;  // Timeout no end
    }
    pong_time = micros();

    // Calcula distância: velocidade do som ~343 m/s, divide por 2 (ida e volta), em cm
    distance = (float)(pong_time - ping_time) * 0.01715;  // ~ (tempo em us * 0.0343 / 2) * 100 para cm

    return (int)distance;
}

int main(int argc, char *argv[]) {
    printf("Teste de sensor HC-SR04 no Raspberry Pi 4\n");
    printf("Trigger: GPIO %d, Echo: GPIO %d\n", TRIG, ECHO);
    printf("Ignorando leituras < %d cm ou > %d cm\n\n", MIN_DISTANCE, MAX_DISTANCE);

    if (wiringPiSetupGpio() == -1) {  // Usa numeração BCM GPIO
        printf("Erro ao inicializar wiringPi!\n");
        return 1;
    }

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    digitalWrite(TRIG, LOW);  // Trigger começa baixo

    while (1) {  // Loop infinito
        int distance = ping();
        if (distance > 0 && distance >= MIN_DISTANCE && distance <= MAX_DISTANCE) {
            printf("Distância válida: %d cm\n", distance);
            // Aqui você pode adicionar lógica para "agir apenas perto", ex: if (distance < 50) { /* faça algo */ }
        } else {
            printf("Leitura inválida (fora de range ou erro)\n");
            // Desconsidera leituras longe ou próximas demais, como pedido
        }
        sleep(1);  // Espera 1 segundo entre leituras
    }

    return 0;
}