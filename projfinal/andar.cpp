#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>
#include <gpiod.h>

using namespace std::chrono_literals;

// Offsets BCM (gpiochip0)
static const unsigned int IN1 = 17; // Motor A
static const unsigned int IN2 = 27; // Motor A
static const unsigned int IN3 = 22; // Motor B
static const unsigned int IN4 = 23; // Motor B

struct Motor {
    gpiod_chip* chip = nullptr;
    gpiod_line* l_in1 = nullptr;
    gpiod_line* l_in2 = nullptr;
    gpiod_line* l_in3 = nullptr;
    gpiod_line* l_in4 = nullptr;

    // FIXO: ambos motores invertidos para adequar ao seu hardware
    bool invA = true;
    bool invB = true;

    bool init(const char* chip_path="/dev/gpiochip0") {
        chip = gpiod_chip_open(chip_path);
        if (!chip) {
            std::fprintf(stderr, "Erro: não foi possível abrir %s (use sudo ou entre no grupo gpio).\n", chip_path);
            return false;
        }

        // pega as linhas
        l_in1 = gpiod_chip_get_line(chip, IN1);
        l_in2 = gpiod_chip_get_line(chip, IN2);
        l_in3 = gpiod_chip_get_line(chip, IN3);
        l_in4 = gpiod_chip_get_line(chip, IN4);
        if (!l_in1 || !l_in2 || !l_in3 || !l_in4) {
            std::fprintf(stderr, "Erro: falha ao obter linhas GPIO.\n");
            return false;
        }

        // solicita como saída com valor inicial 0
        if (gpiod_line_request_output(l_in1, "motor-gpiod", 0) < 0 ||
            gpiod_line_request_output(l_in2, "motor-gpiod", 0) < 0 ||
            gpiod_line_request_output(l_in3, "motor-gpiod", 0) < 0 ||
            gpiod_line_request_output(l_in4, "motor-gpiod", 0) < 0) {
            std::fprintf(stderr, "Erro: falha ao solicitar linhas como saída (ocupadas/permissão?).\n");
            return false;
        }

        parar();
        return true;
    }

    void set_raw(bool a1,bool a2,bool b1,bool b2) {
        // set individual (v1 não tem request_set_values)
        if (gpiod_line_set_value(l_in1, a1?1:0) < 0 ||
            gpiod_line_set_value(l_in2, a2?1:0) < 0 ||
            gpiod_line_set_value(l_in3, b1?1:0) < 0 ||
            gpiod_line_set_value(l_in4, b2?1:0) < 0) {
            std::fprintf(stderr, "Aviso: falha ao setar valores (pinos ocupados?).\n");
        }
    }

    // Aplica inversão por software FIXA (para deixar 'frente' correta no seu setup)
    void set(bool a1,bool a2,bool b1,bool b2) {
        if (invA) std::swap(a1, a2);
        if (invB) std::swap(b1, b2);
        set_raw(a1,a2,b1,b2);
    }

    void parar()     { set(false,false,false,false); }
    void frente()    { set(true,false,  true,false); }   // agora realmente "pra frente"
    void tras()      { set(false,true,  false,true); }
    void girar_esq() { set(true,false,  false,true); }   // A fwd, B rev
    void girar_dir() { set(false,true,  true,false); }   // A rev, B fwd

    ~Motor() {
        // zera pinos
        auto safe_set = [&](gpiod_line* l){ if (l) gpiod_line_set_value(l, 0); };
        safe_set(l_in1); safe_set(l_in2); safe_set(l_in3); safe_set(l_in4);

        auto release = [&](gpiod_line* l){ if (l) gpiod_line_release(l); };
        release(l_in1); release(l_in2); release(l_in3); release(l_in4);

        if (chip) gpiod_chip_close(chip);
    }
};

static void usage(const char* prog) {
    std::printf(
        "Uso: sudo %s <cmd> [tempo_s]\n"
        "cmd = frente | tras | esq | dir | parar | demo\n"
        "Ex.: sudo %s frente 2\n"
        "     sudo %s demo\n", prog, prog, prog
    );
}

int main(int argc, char** argv) {
    std::string cmd = (argc >= 2) ? argv[1] : "demo";
    double t = (argc >= 3) ? std::atof(argv[2]) : 1.0;

    Motor m;
    if (!m.init()) {
        return 1;
    }

    auto run_for = [&](auto fn, double sec){
        fn();
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(sec*1000)));
        m.parar();
    };

    if      (cmd == "frente") run_for([&]{ m.frente(); }, t);
    else if (cmd == "tras")   run_for([&]{ m.tras(); },   t);
    else if (cmd == "esq")    run_for([&]{ m.girar_esq(); }, t);
    else if (cmd == "dir")    run_for([&]{ m.girar_dir(); }, t);
    else if (cmd == "parar")  m.parar();
    else if (cmd == "demo") {
        std::printf("Demo: frente 1.5s, dir 1.0s, tras 1.5s, esq 1.0s\n");
        run_for([&]{ m.frente(); }, 1.5); std::this_thread::sleep_for(200ms);
        run_for([&]{ m.girar_dir(); }, 1.0); std::this_thread::sleep_for(200ms);
        run_for([&]{ m.tras(); },   1.5); std::this_thread::sleep_for(200ms);
        run_for([&]{ m.girar_esq(); }, 1.0);
    } else {
        usage(argv[0]);
        return 2;
    }

    return 0;
}

//g++ -std=c++17 final.cpp -o motor -lgpiod
// como root OU após adicionar seu usuário ao grupo gpio:
//   sudo usermod -aG gpio $USER  (depois faça logout/login)
//sudo ./motor frente 2
//sudo ./motor dir 1
//sudo ./motor tras 1.5
//sudo ./motor parar
//# ou a demo:
//sudo ./motor
