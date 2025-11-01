#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <csignal>

#include <pigpio.h>

// SDK novo da Slamtec (C1/Cx): certifique-se destes headers no -I
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace std::chrono_literals;
using namespace sl;

// ---------------------------------------------------------
// Pinos BCM ligados ao L298N
// ---------------------------------------------------------
const int IN1 = 17; // Motor A (Pino 11)
const int IN2 = 27; // Motor A (Pino 13)
const int IN3 = 22; // Motor B (Pino 15)
const int IN4 = 23; // Motor B (Pino 16)

// (Opcional) inverter sentido por software se necessário
const bool INV_A = false;
const bool INV_B = false;

// ---------------------------------------------------------
// Motores (pigpio simples)
// ---------------------------------------------------------
static inline void setAB(bool a1, bool a2, bool b1, bool b2) {
    // aplica inversão por software
    bool A1 = a1, A2 = a2, B1 = b1, B2 = b2;
    if (INV_A) std::swap(A1, A2);
    if (INV_B) std::swap(B1, B2);
    gpioWrite(IN1, A1 ? 1 : 0);
    gpioWrite(IN2, A2 ? 1 : 0);
    gpioWrite(IN3, B1 ? 1 : 0);
    gpioWrite(IN4, B2 ? 1 : 0);
}

static inline void parar()     { setAB(false,false,false,false); }
static inline void frente()    { setAB(true ,false, true ,false); }
static inline void tras()      { setAB(false,true ,false,true ); }
static inline void girar_esq() { setAB(true ,false, false,true ); } // A fwd, B rev
static inline void girar_dir() { setAB(false,true , true ,false); } // A rev, B fwd

// ---------------------------------------------------------
// Sinal para finalizar limpo com Ctrl+C
// ---------------------------------------------------------
static volatile bool RUNNING = true;
void on_sigint(int) { RUNNING = false; }

// ---------------------------------------------------------
// Helpers LIDAR (SDK novo)
// ---------------------------------------------------------
struct LidarCtx {
    ILidarDriver* lidar = nullptr;
    IChannel* channel = nullptr;

    bool connectSerial(const char* dev = "/dev/ttyUSB0", uint32_t baud = 460800) {
        // Cria driver
        lidar = *createLidarDriver();
        if (!lidar) {
            std::fprintf(stderr, "Falha: createLidarDriver retornou nullptr\n");
            return false;
        }

        // Cria canal serial (API nova retorna Result<IChannel*>)
        auto ch_res = createSerialPortChannel(dev, baud);
        // Verifica status (Result tem conversão para sl_result)
        if (SL_IS_FAIL((sl_result)ch_res)) {
            std::fprintf(stderr, "Falha: createSerialPortChannel(%s) status=%08x\n",
                         dev, (sl_result)ch_res);
            return false;
        }
        channel = ch_res.value(); // pega o ponteiro IChannel*

        // Abre canal (opcional em alguns devices, seguro chamar)
        if (SL_IS_FAIL(channel->open())) {
            std::fprintf(stderr, "Falha ao abrir porta serial %s\n", dev);
            return false;
        }

        // Conecta o driver
        sl_result res = lidar->connect(channel);
        if (SL_IS_FAIL(res)) {
            std::fprintf(stderr, "Falha ao conectar LIDAR: %08x\n", res);
            return false;
        }

        // Verifica saúde
        sl_lidar_response_device_health_t health{};
        res = lidar->getHealth(health);
        if (SL_IS_FAIL(res)) {
            std::fprintf(stderr, "Falha ao obter health: %08x\n", res);
            return false;
        }
        if (health.status == SL_LIDAR_STATUS_ERROR) {
            std::fprintf(stderr, "LIDAR reportou erro de saúde. Requer manutenção.\n");
            return false;
        }

        // Inicia varredura no primeiro modo disponível
        std::vector<LidarScanMode> modes;
        res = lidar->getAllSupportedScanModes(modes);
        if (SL_IS_FAIL(res) || modes.empty()) {
            std::fprintf(stderr, "Falha ao consultar scan modes: %08x\n", res);
            return false;
        }
        // modo padrão (índice 0 costuma ser ok no C1)
        res = lidar->startScan(false, modes[0].id);
        if (SL_IS_FAIL(res)) {
            std::fprintf(stderr, "Falha ao iniciar scan: %08x\n", res);
            return false;
        }

        return true;
    }

    void stop() {
        if (lidar) {
            lidar->stop();
        }
    }

    ~LidarCtx() {
        if (lidar) {
            // stop() já chamado no fluxo normal; chamar de novo é seguro
            lidar->stop();
            delete lidar;
            lidar = nullptr;
        }
        if (channel) {
            // Alguns canais fecham dentro do destrutor; chamar close() é seguro
            channel->close();
            delete channel;
            channel = nullptr;
        }
    }
};

// ---------------------------------------------------------
// Lógica de evitar obstáculo
// Considera obstáculo à frente se ângulo ∈ [315°, 360°) ∪ [0°, 45°] e dist < 0.30 m
// ---------------------------------------------------------
static inline bool obstaculo_a_frente(ILidarDriver* lidar, float limite_m = 0.30f) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    sl_result res = lidar->grabScanDataHq(nodes, count);
    if (SL_IS_FAIL(res)) {
        // Se falhar leitura, melhor retornar "sem obstáculo" e tentar de novo
        return false;
    }
    lidar->ascendScanData(nodes, count);

    for (size_t i = 0; i < count; ++i) {
        // quality == 0 => leitura inválida
        if (nodes[i].quality == 0) continue;

        // ângulo em graus (ver docs: angle_z_q14 * 90 / 16384)
        float angle_deg = nodes[i].angle_z_q14 * 90.f / 16384.f;
        if (angle_deg < 0.f) continue;

        // distância em metros (dist_mm_q2 / 4 = mm; /1000 => m)
        float dist_m = (nodes[i].dist_mm_q2 / 4.0f) / 1000.0f;
        if (dist_m <= 0.f) continue;

        bool frente_setor =
            (angle_deg <= 45.f) || (angle_deg >= 315.f); // [-45°,45°] normalizado em [0,360)

        if (frente_setor && dist_m < limite_m) {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------
// Execução
// ---------------------------------------------------------
static void usage(const char* prog) {
    std::printf(
        "Uso: sudo %s <cmd> [tempo_s]\n"
        "cmd = frente | tras | esq | dir | parar | demo | auto\n"
        "Ex.: sudo %s frente 2\n"
        "     sudo %s auto\n", prog, prog, prog
    );
}

int main(int argc, char** argv) {
    // Trata Ctrl+C pra sair limpo
    std::signal(SIGINT, on_sigint);

    if (gpioInitialise() < 0) { // precisa rodar com sudo (ou pigpio como daemon)
        std::fprintf(stderr, "Falha ao inicializar pigpio (rode com sudo)\n");
        return 1;
    }
    gpioSetMode(IN1, PI_OUTPUT);
    gpioSetMode(IN2, PI_OUTPUT);
    gpioSetMode(IN3, PI_OUTPUT);
    gpioSetMode(IN4, PI_OUTPUT);
    parar();

    double t = (argc >= 3) ? std::atof(argv[2]) : 1.0;
    std::string cmd = (argc >= 2) ? argv[1] : "demo";

    auto run_for = [&](void(*fn)(), double sec) {
        fn();
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(sec * 1000)));
        parar();
    };

    if (cmd == "frente")         run_for(frente, t);
    else if (cmd == "tras")      run_for(tras, t);
    else if (cmd == "esq")       run_for(girar_esq, t);
    else if (cmd == "dir")       run_for(girar_dir, t);
    else if (cmd == "parar")     parar();
    else if (cmd == "auto") {
        std::printf("Modo autônomo: seguir em frente e desviar de obstáculos < 0.30 m\n");

        LidarCtx ctx;
        if (!ctx.connectSerial("/dev/ttyUSB0", 460800)) {
            std::fprintf(stderr, "Não foi possível inicializar o LIDAR.\n");
            gpioTerminate();
            return 2;
        }

        // loop principal
        while (RUNNING) {
            // tenta ir pra frente
            frente();

            // checa obstáculo à frente
            bool tem = obstaculo_a_frente(ctx.lidar, 0.30f);
            if (tem) {
                // estratégia simples: para, gira à esquerda 0.8s, tenta novamente
                parar();
                std::this_thread::sleep_for(150ms);

                girar_esq();
                std::this_thread::sleep_for(800ms);

                parar();
                std::this_thread::sleep_for(150ms);
            }

            std::this_thread::sleep_for(50ms);
        }

        // parar scan e motores
        ctx.stop();
        parar();
    }
    else if (cmd == "demo") {
        std::printf("Demo: frente 1.5s, dir 1.0s, tras 1.5s, esq 1.0s\n");
        run_for(frente, 1.5); std::this_thread::sleep_for(200ms);
        run_for(girar_dir, 1.0); std::this_thread::sleep_for(200ms);
        run_for(tras, 1.5); std::this_thread::sleep_for(200ms);
        run_for(girar_esq, 1.0);
    }
    else {
        usage(argv[0]);
        gpioTerminate();
        return 3;
    }

    gpioTerminate();
    return 0;
}


sudo apt-get update
sudo apt-get install -y g++ pigpio

# Exemplo: se o SDK estiver em ../rplidar_sdk
g++ -std=c++17 motor_lidar.cpp -o motor_lidar \
  -I../rplidar_sdk/sdk/include \
  -L../rplidar_sdk/output/Linux/Release \
  -lsl_lidar_sdk -lpigpio -lrt -lpthread

