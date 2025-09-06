#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <cstdlib>

// ====== CONFIGURE SEUS PINOS (BCM) ======
static const int IN1 = 17; // Motor A
static const int IN2 = 27; // Motor A
static const int IN3 = 22; // Motor B
static const int IN4 = 23; // Motor B
// Se seu L298N não tiver jumpers em ENA/ENB, defina pinos para habilitar:
static const int ENA = -1; // ex.: 12 (ou -1 para não usar)
static const int ENB = -1; // ex.: 13 (ou -1 para não usar)
// ========================================

using namespace std::chrono_literals;

static bool write_file(const std::string& path, const std::string& data) {
    std::ofstream f(path);
    if (!f) return false;
    f << data;
    return true;
}

static bool export_gpio(int pin) {
    if (pin < 0) return true;
    // já exportado? então ok
    std::ifstream chk("/sys/class/gpio/gpio" + std::to_string(pin) + "/value");
    if (chk.good()) return true;
    return write_file("/sys/class/gpio/export", std::to_string(pin));
}

static bool unexport_gpio(int pin) {
    if (pin < 0) return true;
    return write_file("/sys/class/gpio/unexport", std::to_string(pin));
}

static bool set_dir(int pin, const std::string& dir) {
    if (pin < 0) return true;
    return write_file("/sys/class/gpio/gpio" + std::to_string(pin) + "/direction", dir);
}

static bool set_val(int pin, int val) {
    if (pin < 0) return true;
    return write_file("/sys/class/gpio/gpio" + std::to_string(pin) + "/value", val ? "1" : "0");
}

static void gpio_init() {
    for (int p : {IN1, IN2, IN3, IN4, ENA, ENB}) {
        export_gpio(p);
        set_dir(p, "out");
        set_val(p, 0);
    }
    // habilita ENA/ENB se definidos
    if (ENA >= 0) set_val(ENA, 1);
    if (ENB >= 0) set_val(ENB, 1);
}

static void gpio_cleanup() {
    for (int p : {IN1, IN2, IN3, IN4, ENA, ENB}) {
        set_val(p, 0);
        unexport_gpio(p);
    }
}

// ====== MOVIMENTOS ======
static void parar() {
    set_val(IN1,0); set_val(IN2,0); set_val(IN3,0); set_val(IN4,0);
}
static void frente() {
    // A: IN1=1 IN2=0  |  B: IN3=1 IN4=0
    set_val(IN1,1); set_val(IN2,0); set_val(IN3,1); set_val(IN4,0);
}
static void tras() {
    set_val(IN1,0); set_val(IN2,1); set_val(IN3,0); set_val(IN4,1);
}
static void girar_esq() {
    // A frente, B ré
    set_val(IN1,1); set_val(IN2,0); set_val(IN3,0); set_val(IN4,1);
}
static void girar_dir() {
    // A ré, B frente
    set_val(IN1,0); set_val(IN2,1); set_val(IN3,1); set_val(IN4,0);
}

static void run_for(void (*fn)(), double seconds) {
    fn();
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(seconds*1000)));
    parar();
}

// ====== MAIN ======
int main(int argc, char** argv) {
    if (geteuid() != 0) {
        std::cerr << "Execute com sudo (necessário para /sys/class/gpio).\n";
        return 1;
    }

    gpio_init();

    double t = (argc >= 3) ? std::atof(argv[2]) : 1.0;
    std::string cmd = (argc >= 2) ? argv[1] : "demo";

    try {
        if (cmd == "frente" || cmd == "forward") {
            run_for(frente, t);
        } else if (cmd == "tras" || cmd == "re" || cmd == "back") {
            run_for(tras, t);
        } else if (cmd == "esq" || cmd == "left" || cmd == "girar_esq") {
            run_for(girar_esq, t);
        } else if (cmd == "dir" || cmd == "right" || cmd == "girar_dir") {
            run_for(girar_dir, t);
        } else if (cmd == "parar" || cmd == "stop") {
            parar();
        } else {
            // demo: frente -> parar -> direita -> parar -> tras -> parar -> esquerda
            std::cout << "Demo: frente 1.5s, dir 1.0s, tras 1.5s, esq 1.0s\n";
            run_for(frente, 1.5);
            std::this_thread::sleep_for(200ms);
            run_for(girar_dir, 1.0);
            std::this_thread::sleep_for(200ms);
            run_for(tras, 1.5);
            std::this_thread::sleep_for(200ms);
            run_for(girar_esq, 1.0);
        }
    } catch (...) {
        parar();
    }

    gpio_cleanup();
    return 0;
}
