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
    gpiod_line_settings* st = nullptr;
    gpiod_line_config* lcfg = nullptr;
    gpiod_request_config* rcfg = nullptr;
    gpiod_line_request* req = nullptr;

    unsigned int offsets[4] = {IN1, IN2, IN3, IN4};
    gpiod_line_value vals[4] = {
        GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE,
        GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE
    };

    bool init(const char* chip_path="/dev/gpiochip0") {
        chip = gpiod_chip_open(chip_path);
        if (!chip) {
            std::fprintf(stderr, "Erro: não foi possível abrir %s (use sudo ou entre no grupo gpio).\n", chip_path);
            return false;
        }

        st = gpiod_line_settings_new();
        if (!st) { std::fprintf(stderr, "Erro: gpiod_line_settings_new\n"); return false; }
        gpiod_line_settings_set_direction(st, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(st, GPIOD_LINE_VALUE_INACTIVE);

        lcfg = gpiod_line_config_new();
        if (!lcfg) { std::fprintf(stderr, "Erro: gpiod_line_config_new\n"); return false; }
        if (gpiod_line_config_add_line_settings(lcfg, offsets, 4, st) < 0) {
            std::fprintf(stderr, "Erro: gpiod_line_config_add_line_settings\n");
            return false;
        }

        rcfg = gpiod_request_config_new();
        if (!rcfg) { std::fprintf(stderr, "Erro: gpiod_request_config_new\n"); return false; }
        gpiod_request_config_set_consumer(rcfg, "motor-gpiod");

        req = gpiod_chip_request_lines(chip, rcfg, lcfg);
        if (!req) {
            std::fprintf(stderr, "Erro: gpiod_chip_request_lines (verifique permissões e se os pinos não estão em uso)\n");
            return false;
        }

        parar();
        return true;
    }

    void set(bool a1,bool a2,bool b1,bool b2) {
        vals[0] = a1 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE;
        vals[1] = a2 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE;
        vals[2] = b1 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE;
        vals[3] = b2 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE;
        // API v2: SEM o parâmetro de contagem
        if (gpiod_line_request_set_values(req, vals) < 0) {
            std::fprintf(stderr, "Aviso: falha ao setar valores (pinos ocupados?)\n");
        }
    }

    void parar()     { set(false,false,false,false); }
    void frente()    { set(true,false,  true,false); }
    void tras()      { set(false,true,  false,true); }
    void girar_esq() { set(true,false,  false,true); }
    void girar_dir() { set(false,true,  true,false); }

    ~Motor() {
        // Garante parada
        if (req) {
            vals[0]=vals[1]=vals[2]=vals[3]=GPIOD_LINE_VALUE_INACTIVE;
            gpiod_line_request_set_values(req, vals);
        }
        if (req)  { gpiod_line_request_release(req); }
        if (rcfg) { gpiod_request_config_free(rcfg); }
        if (lcfg) { gpiod_line_config_free(lcfg); }
        if (st)   { gpiod_line_settings_free(st); }
        if (chip) { gpiod_chip_close(chip); }
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
