#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <gpiod.h>

using namespace std::chrono_literals;

// Direção (TB6612FNG) - BCM
static const unsigned int AIN1 = 17; // Motor A
static const unsigned int AIN2 = 27; // Motor A
static const unsigned int BIN1 = 22; // Motor B
static const unsigned int BIN2 = 23; // Motor B

struct Motor {
    gpiod_chip* chip = nullptr;
    gpiod_line_settings* st = nullptr;
    gpiod_line_config* lcfg = nullptr;
    gpiod_request_config* rcfg = nullptr;
    gpiod_line_request* req = nullptr;

    // Inverta se "frente" sair ao contrário
    bool invA = true;
    bool invB = true;

    unsigned int offsets[4] = { AIN1, AIN2, BIN1, BIN2 };
    gpiod_line_value vals[4] = {
        GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE,
        GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE
    };

    bool init(const char* chip_path) {
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
        gpiod_request_config_set_consumer(rcfg, "tb6612-4wires");

        req = gpiod_chip_request_lines(chip, rcfg, lcfg);
        if (!req) {
            std::fprintf(stderr, "Erro: gpiod_chip_request_lines (permissões? pinos em uso?)\n");
            return false;
        }

        parar();
        return true;
    }

    void set_raw(bool a1,bool a2,bool b1,bool b2) {
        vals[0] = a1 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE; // AIN1
        vals[1] = a2 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE; // AIN2
        vals[2] = b1 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE; // BIN1
        vals[3] = b2 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE; // BIN2
        if (gpiod_line_request_set_values(req, vals) < 0) {
            std::fprintf(stderr, "Aviso: falha ao setar direções (pinos ocupados?)\n");
        }
    }

    void set(bool a1,bool a2,bool b1,bool b2) {
        if (invA) std::swap(a1, a2);
        if (invB) std::swap(b1, b2);
        set_raw(a1,a2,b1,b2);
    }

    void parar()     { set(false,false, false,false); }
    void frente()    { set(true,false,  true,false); }
    void tras()      { set(false,true,  false,true); }
    void girar_esq() { set(true,false,  false,true); }
    void girar_dir() { set(false,true,  true,false); }

    ~Motor() {
        if (req) {
            vals[0]=vals[1]=vals[2]=vals[3]=GPIOD_LINE_VALUE_INACTIVE;
            gpiod_line_request_set_values(req, vals);
            gpiod_line_request_release(req);
        }
        if (rcfg) gpiod_request_config_free(rcfg);
        if (lcfg) gpiod_line_config_free(lcfg);
        if (st)   gpiod_line_settings_free(st);
        if (chip) gpiod_chip_close(chip);
    }
};

static void usage(const char* prog) {
    std::printf(
        "Uso: sudo %s <cmd> [tempo_s] [--chip /dev/gpiochipN]\n"
        "cmd = frente | tras | esq | dir | parar | demo\n"
        "Ex.: sudo %s frente 2 --chip /dev/gpiochip4\n"
        "     sudo %s demo\n", prog, prog, prog
    );
}

int main(int argc, char** argv) {
    std::string cmd = (argc >= 2) ? argv[1] : "demo";
    double t = (argc >= 3 && argv[2][0] != '-') ? std::atof(argv[2]) : 1.0;

    // Default bom para Raspberry Pi 5:
    const char* chip_path = "/dev/gpiochip4";
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--chip") == 0 && i+1 < argc) {
            chip_path = argv[i+1];
        }
    }

    Motor m;
    if (!m.init(chip_path)) return 1;

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


// Compilar:
// g++ -std=c++17 motor_tb6612_4pins.cpp -o motor_tb6612_4pins -lgpiod
//
// Executar (como root ou usuário no grupo gpio):
//   sudo ./motor_tb6612_4pins frente 2
//   sudo ./motor_tb6612_4pins dir 1
//   sudo ./motor_tb6612_4pins tras 1.5
//   sudo ./motor_tb6612_4pins parar
//   sudo ./motor_tb6612_4pins demo


// Compilar:
// g++ -std=c++17 motor_tb6612.cpp -o motor_tb6612 -lgpiod
//
// Permissões (uma vez):
//   sudo usermod -aG gpio $USER   # faça logout/login depois
//
// Executar (exemplos):
//   sudo ./motor_tb6612 frente 2
//   sudo ./motor_tb6612 dir 1
//   sudo ./motor_tb6612 tras 1.5
//   sudo ./motor_tb6612 parar
//   sudo ./motor_tb6612 demo
