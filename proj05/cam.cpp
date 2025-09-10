#include <iostream>
#include <wiringPi.h>
#include <opencv2/opencv.hpp>

#define TRIG 5
#define ECHO 6

// Função para medir distância com ultrassônico
long getDistance() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    while(digitalRead(ECHO) == LOW);
    long startTime = micros();
    while(digitalRead(ECHO) == HIGH);
    long travelTime = micros() - startTime;

    long distance = travelTime * 0.034 / 2; // cm
    return distance;
}

int main() {
    wiringPiSetup();
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    cv::VideoCapture cap(0); // sua camera já configurada
    if (!cap.isOpened()) {
        std::cerr << "Erro: não abriu a câmera.\n";
        return -1;
    }

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        long dist = getDistance();
        std::cout << "Distancia: " << dist << " cm\n";

        if (dist < 30) {
            std::cout << "⚠️ Obstáculo detectado! Desviando...\n";
            // Aqui você chama função para parar/girar motores
        }

        cv::imshow("Camera", frame);
        if (cv::waitKey(1) == 27) break; // ESC para sair
    }
    return 0;
}
