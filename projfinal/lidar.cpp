#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include <gpiod.h>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace std::chrono_literals;
using namespace sl;

// ---- GPIO (BCM offsets no gpiochip0) ----
static const unsigned int IN1 = 17;
static const unsigned int IN2 = 27;
static const unsigned int IN3 = 22;
static const unsigned int IN4 = 23;

// Inversão por software para corrigir o seu hardware
static const bool INV_A = true;  // <-- ATIVADO
static const bool INV_B = true;  // <-- ATIVADO

// libgpiod v1 handles
static gpiod_chip* chip = nullptr;
static gpiod_line* L1   = nullptr;
static gpiod_line* L2   = nullptr;
static gpiod_line* L3   = nullptr;
static gpiod_line* L4   = nullptr;

static bool gpio_init(const char* chip_path="/dev/gpiochip0") {
    chip = gpiod_chip_open(chip_path);
    if (!chip) {
        std::fprintf(stderr, "Erro: não abriu %s (use sudo ou entre no grupo gpio)\n", chip_path);
        return false;
    }

    L1 = gpiod_chip_get_line(chip, IN1);
    L2 = gpiod_chip_get_line(chip, IN2);
    L3 = gpiod_chip_get_line(chip, IN3);
    L4 = gpiod_chip_get_line(chip, IN4);
    if (!L1 || !L2 || !L3 || !L4) {
        std::fprintf(stderr, "Erro: falha ao obter linhas do gpiochip.\n");
        return false;
    }

    if (gpiod_line_request_output(L1, "amr-motors", 0) < 0 ||
        gpiod_line_request_output(L2, "amr-motors", 0) < 0 ||
        gpiod_line_request_output(L3, "amr-motors", 0) < 0 ||
        gpiod_line_request_output(L4, "amr-motors", 0) < 0) {
        std::fprintf(stderr, "Erro: falha ao solicitar linhas como saída (ocupadas/permissão?).\n");
        return false;
    }

    // tudo OFF
    gpiod_line_set_value(L1, 0);
    gpiod_line_set_value(L2, 0);
    gpiod_line_set_value(L3, 0);
    gpiod_line_set_value(L4, 0);
    return true;
}

static void gpio_close() {
    auto safe_off = [](gpiod_line* L){ if (L) gpiod_line_set_value(L, 0); };
    safe_off(L1); safe_off(L2); safe_off(L3); safe_off(L4);

    auto release = [](gpiod_line* L){ if (L) gpiod_line_release(L); };
    release(L1); release(L2); release(L3); release(L4);

    if (chip) gpiod_chip_close(chip);
    chip = nullptr; L1 = L2 = L3 = L4 = nullptr;
}

// ---- Controle motores (L298N) - v1 usa set individual ----
static inline void set_raw(bool a1,bool a2,bool b1,bool b2) {
    if (gpiod_line_set_value(L1, a1?1:0) < 0 ||
        gpiod_line_set_value(L2, a2?1:0) < 0 ||
        gpiod_line_set_value(L3, b1?1:0) < 0 ||
        gpiod_line_set_value(L4, b2?1:0) < 0) {
        std::fprintf(stderr, "Aviso: falha ao setar GPIO (ocupado?)\n");
    }
}

static inline void set(bool a1,bool a2,bool b1,bool b2){
    bool A1=a1, A2=a2, B1=b1, B2=b2;
    if (INV_A) std::swap(A1, A2);
    if (INV_B) std::swap(B1, B2);
    set_raw(A1,A2,B1,B2);
}

static inline void parar()     { set(false,false,false,false); }
static inline void frente()    { set(true ,false, true ,false); } // agora REALMENTE "frente"
static inline void tras()      { set(false,true ,false,true ); }
static inline void girar_esq() { set(true ,false, false,true ); } // A fwd, B rev
static inline void girar_dir() { set(false,true , true ,false); } // A rev, B fwd

// ---- LIDAR ----
struct LidarCtx {
    ILidarDriver* lidar {nullptr};
    IChannel*     channel {nullptr};

    bool connectSerial(const char* dev="/dev/ttyUSB0", uint32_t baud=460800) {
        lidar = *createLidarDriver();
        if (!lidar) { std::fprintf(stderr, "createLidarDriver nullptr\n"); return false; }

        auto ch_res = createSerialPortChannel(dev, baud); // Result<IChannel*>
        sl_result rc = ch_res;                            // usa operator sl_result()
        if (SL_IS_FAIL(rc)) {
            std::fprintf(stderr, "createSerialPortChannel(%s) falhou: %08x\n", dev, rc);
            return false;
        }
        channel = ch_res.value;                           // pega o ponteiro
        if (!channel) { std::fprintf(stderr, "channel nullptr\n"); return false; }

        sl_result res = lidar->connect(channel);
        if (SL_IS_FAIL(res)) {
            std::fprintf(stderr, "connect falhou: %08x\n", res);
            return false;
        }

        sl_lidar_response_device_health_t health{};
        res = lidar->getHealth(health);
        if (SL_IS_FAIL(res) || health.status == SL_LIDAR_STATUS_ERROR) {
            std::fprintf(stderr, "health erro: %08x, status=%u\n", res, health.status);
            return false;
        }

        std::vector<LidarScanMode> modes;
        res = lidar->getAllSupportedScanModes(modes);
        if (SL_IS_FAIL(res) || modes.empty()) {
            std::fprintf(stderr, "getAllSupportedScanModes falhou: %08x\n", res);
            return false;
        }
        res = lidar->startScan(false, modes[0].id);
        if (SL_IS_FAIL(res)) {
            std::fprintf(stderr, "startScan falhou: %08x\n", res);
            return false;
        }
        return true;
    }

    void stop() { if (lidar) lidar->stop(); }

    ~LidarCtx() {
        if (lidar) { lidar->stop(); delete lidar; lidar=nullptr; }
        if (channel) { delete channel; channel=nullptr; }
    }
};

// obstáculo: frente = [315°,360°) ∪ [0°,45°], dist < 0.30 m
static inline bool obstaculo_a_frente(ILidarDriver* lidar, float limite_m=0.30f) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes)/sizeof(nodes[0]);

    sl_result r = lidar->grabScanDataHq(nodes, count);
    if (SL_IS_FAIL(r)) return false;
    lidar->ascendScanData(nodes, count);

    for (size_t i=0;i<count;++i) {
        if (nodes[i].quality == 0) continue;
        float ang   = nodes[i].angle_z_q14 * 90.f / 16384.f;   // graus
        float distm = (nodes[i].dist_mm_q2 / 4.0f) / 1000.0f;  // metros
        if (distm <= 0.f) continue;
        bool frente = (ang <= 45.f) || (ang >= 315.f);
        if (frente && distm < limite_m) return true;
    }
    return false;
}

// ---- CLI / Main ----
static volatile bool RUNNING = true;
static void on_sigint(int){ RUNNING=false; }

static void usage(const char* prog){
    std::printf(
        "Uso: sudo %s <cmd> [tempo_s]\n"
        "cmd = frente | tras | esq | dir | parar | demo | auto\n"
        "Ex.: sudo %s frente 2\n"
        "     sudo %s auto\n", prog, prog, prog
    );
}

int main(int argc, char** argv) {
    std::signal(SIGINT, on_sigint);

    if (!gpio_init()) return 1;

    std::string cmd = (argc>=2)? argv[1] : "auto";
    double t = (argc>=3)? std::atof(argv[2]) : 1.0;

    auto run_for = [&](auto fn, double sec){
        fn();
        std::this_thread::sleep_for(std::chrono::milliseconds(int(sec*1000)));
        parar();
    };

    if      (cmd=="frente") run_for(frente,t);
    else if (cmd=="tras")   run_for(tras,t);
    else if (cmd=="esq")    run_for(girar_esq,t);
    else if (cmd=="dir")    run_for(girar_dir,t);
    else if (cmd=="parar")  parar();
    else if (cmd=="demo") {
        std::printf("Demo: frente 1.5s, dir 1.0s, tras 1.5s, esq 1.0s\n");
        run_for(frente,1.5); std::this_thread::sleep_for(200ms);
        run_for(girar_dir,1.0); std::this_thread::sleep_for(200ms);
        run_for(tras,1.5);   std::this_thread::sleep_for(200ms);
        run_for(girar_esq,1.0);
    }
    else if (cmd=="auto") {
        std::printf("Auto: evitar obstáculo < 0.30 m na frente\n");
        LidarCtx ctx;
        if (!ctx.connectSerial("/dev/ttyUSB0", 460800)) {
            std::fprintf(stderr, "LIDAR não inicializou. (Tente 256000 ou 115200 se necessário)\n");
            gpio_close();
            return 2;
        }
        while (RUNNING) {
            frente();
            if (obstaculo_a_frente(ctx.lidar, 0.30f)) {
                parar(); std::this_thread::sleep_for(150ms);
                girar_esq(); std::this_thread::sleep_for(800ms);
                parar(); std::this_thread::sleep_for(150ms);
            }
            std::this_thread::sleep_for(50ms);
        }
        ctx.stop();
        parar();
    }
    else {
        usage(argv[0]);
        gpio_close();
        return 3;
    }

    gpio_close();
    return 0;
}


# ajuste os caminhos do SDK do RPLIDAR conforme sua pasta
g++ -std=c++17 amr_lidar_v1.cpp -o amr \
  -I../rplidar_sdk/sdk/include \
  -L../rplidar_sdk/output/Linux/Release \
  -lsl_lidar_sdk -lgpiod -pthread



# Dê permissão (ou use sudo)
sudo usermod -aG gpio $USER   # faça logout/login depois

# Testes:
sudo ./amr demo
sudo ./amr frente 2
sudo ./amr dir 1
sudo ./amr tras 1.5
sudo ./amr parar

# Modo automático (desvia se obstáculo < 0.30 m):
sudo ./amr auto  