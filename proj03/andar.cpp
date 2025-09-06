#include <cstdio>
#include <cstdlib>
#include <pigpio.h>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

// BCM pins
const int IN1 = 17; // A
const int IN2 = 27; // A
const int IN3 = 22; // B
const int IN4 = 23; // B

void parar()       { gpioWrite(IN1,0); gpioWrite(IN2,0); gpioWrite(IN3,0); gpioWrite(IN4,0); }
void frente()      { gpioWrite(IN1,1); gpioWrite(IN2,0); gpioWrite(IN3,1); gpioWrite(IN4,0); }
void tras()        { gpioWrite(IN1,0); gpioWrite(IN2,1); gpioWrite(IN3,0); gpioWrite(IN4,1); }
void girar_esq()   { gpioWrite(IN1,1); gpioWrite(IN2,0); gpioWrite(IN3,0); gpioWrite(IN4,1); }
void girar_dir()   { gpioWrite(IN1,0); gpioWrite(IN2,1); gpioWrite(IN3,1); gpioWrite(IN4,0); }

int main(int argc, char** argv) {
    if (gpioInitialise() < 0) { // inicia a lib
        fprintf(stderr, "pigpio init failed\n");
        return 1;
    }
    gpioSetMode(IN1, PI_OUTPUT);
    gpioSetMode(IN2, PI_OUTPUT);
    gpioSetMode(IN3, PI_OUTPUT);
    gpioSetMode(IN4, PI_OUTPUT);
    parar();

    double t = (argc >= 3) ? atof(argv[2]) : 1.0;
    const char* cmd = (argc >= 2) ? argv[1] : "demo";

    auto run_for = [&](void(*fn)(), double sec){
        fn();
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(sec*1000)));
        parar();
    };

    if      (std::string(cmd) == "frente") run_for(frente, t);
    else if (std::string(cmd) == "tras")   run_for(tras,   t);
    else if (std::string(cmd) == "esq")    run_for(girar_esq, t);
    else if (std::string(cmd) == "dir")    run_for(girar_dir, t);
    else if (std::string(cmd) == "parar")  parar();
    else {
        printf("Demo: frente 1.5s, dir 1.0s, tras 1.5s, esq 1.0s\n");
        run_for(frente, 1.5);
        std::this_thread::sleep_for(200ms);
        run_for(girar_dir, 1.0);
        std::this_thread::sleep_for(200ms);
        run_for(tras, 1.5);
        std::this_thread::sleep_for(200ms);
        run_for(girar_esq, 1.0);
    }

    parar();
    gpioTerminate();
    return 0;
}
