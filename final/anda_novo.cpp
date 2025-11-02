
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>
#include <gpiod.h>

using namespace std::chrono_literals;

// ===== TB6612FNG - GPIO (BCM) =====
// Motor A (direção)
static const unsigned int AIN1 = 17;
static const unsigned int AIN2 = 27;
// Motor B (direção)
static const unsigned int BIN1 = 22;
static const unsigned int BIN2 = 23;
// PWM (aqui apenas mantidos em HIGH para “habilitar sempre”)
static const unsigned int PWMA = 18;  // pode ser PWM hardware no futuro
static const unsigned int PWMB = 13;  // pode ser PWM hardware no futuro
// Standby
static const unsigned int STBY = 24;

struct Motor {
    gpiod_chip* chip = nullptr;
    gpiod_line_settings* st = nullptr;
    gpiod_line_config* lcfg = nullptr;
    gpiod_request_config* rcfg = nullptr;
    gpiod_line_request* req = nullptr;

    // Inversões por software (mantenho como TRUE se seu hardware estiver invertido)
    bool invA = true;
    bool invB = true;

    // offsets: ordem fixa -> [AIN1, AIN2, BIN1, BIN2, PWMA, PWMB, STBY]
    unsigned int offsets[7] = { AIN1, AIN2, BIN1, BIN2, PWMA, PWMB, STBY };

    // valores de saída correspondentes aos offsets acima
    gpiod_line_value vals[7] = {
        GPIOD_LINE_VALUE_INACTIVE, // AIN1
        GPIOD_LINE_VALUE_INACTIVE, // AIN2
        GPIOD_LINE_VALUE_INACTIVE, // BIN1
        GPIOD_LINE_VALUE_INACTIVE, // BIN2
        GPIOD_LINE_VALUE_INACTIVE, // PWMA
        GPIOD_LINE_VALUE_INACTIVE, // PWMB
        GPIOD_LINE_VALUE_INACTIVE  // STBY
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
        if (gpiod_line_config_add_line_settings(lcfg, offsets, 7, st) < 0) {
            std::fprintf(stderr, "Erro: gpiod_line_config_add_line_settings\n");
            return false;
        }

        rcfg = gpiod_request_config_new();
        if (!rcfg) { std::fprintf(stderr, "Erro: gpiod_request_config_new\n"); return false; }
        gpiod_request_config_set_consumer(rcfg, "tb6612-gpiod");

        req = gpiod_chip_request_lines(chip, rcfg, lcfg);
        if (!req) {
            std::fprintf(stderr, "Erro: gpiod_chip_request_lines (verifique permissões e se os pinos não estão em uso)\n");
            return false;
        }

        // Coloca tudo em nível seguro
        parar();

        // Habilita o driver: PWMA=HIGH, PWMB=HIGH, STBY=HIGH
        enable_driver(true);

        return true;
    }

    void enable_driver(bool en) {
        vals[4] = en ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE; // PWMA
        vals[5] = en ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE; // PWMB
        vals[6] = en ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE; // STBY
        if (gpiod_line_request_set_values(req, vals) < 0) {
            std::fprintf(stderr, "Aviso: falha ao setar PWMA/PWMB/STBY (pinos ocupados?)\n");
        }
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

    // Aplica inversão por software para “frente” ficar correta no seu robô
    void set(bool a1,bool a2,bool b1,bool b2) {
        if (invA) std::swap(a1, a2);
        if (invB) std::swap(b1, b2);
        set_raw(a1,a2,b1,b2);
    }

    // Movimentos básicos (com PWMA/PWMB já em HIGH)
    void parar()     { set(false,false, false,false); }
    void frente()    { set(true,false,  true,false); }   // A fwd, B fwd
    void tras()      { set(false,true,  false,true); }   // A rev, B rev
    void girar_esq() { set(true,false,  false,true); }   // A fwd, B rev
    void girar_dir() { set(false,true,  true,false); }   // A rev, B fwd

    ~Motor() {
        if (req) {
            // desabilita: para motores e coloca PWMs/STBY baixos
            vals[0]=vals[1]=vals[2]=vals[3]=GPIOD_LINE_VALUE_INACTIVE;
            vals[4]=vals[5]=vals[6]=GPIOD_LINE_VALUE_INACTIVE;
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
