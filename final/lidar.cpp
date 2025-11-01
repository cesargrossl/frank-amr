#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <csignal>

#include <pigpio.h>

// Headers do SDK Slamtec (C1/Cx)
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace std::chrono_literals;
using namespace sl;

// GPIO L298N (BCM)
const int IN1 = 17;
const int IN2 = 27;
const int IN3 = 22;
const int IN4 = 23;

const bool INV_A = false;
const bool INV_B = false;

static inline void setAB(bool a1, bool a2, bool b1, bool b2) {
    bool A1=a1, A2=a2, B1=b1, B2=b2;
    if (INV_A) std::swap(A1, A2);
    if (INV_B) std::swap(B1, B2);
    gpioWrite(IN1, A1); gpioWrite(IN2, A2);
    gpioWrite(IN3, B1); gpioWrite(IN4, B2);
}
static inline void parar()     { setAB(false,false,false,false); }
static inline void frente()    { setAB(true ,false, true ,false); }
static inline void tras()      { setAB(false,true ,false,true ); }
static inline void girar_esq() { setAB(true ,false, false,true ); }
static inline void girar_dir() { setAB(false,true , true ,false); }

static volatile bool RUNNING = true;
void on_sigint(int){ RUNNING=false; }

struct LidarCtx {
    ILidarDriver* lidar = nullptr;
    IChannel* channel   = nullptr;

    bool connectSerial(const char* dev="/dev/ttyUSB0", uint32_t baud=460800) {
        lidar = *createLidarDriver();
        if (!lidar) {
            std::fprintf(stderr, "Falha: createLidarDriver retornou nullptr\n");
            return false;
        }

        // >>> SUA VERSÃO DO SDK: retorna IChannel* direto <<<
        channel = createSerialPortChannel(dev, baud);
        if (!channel) {
            std::fprintf(stderr, "Falha: createSerialPortChannel(%s) retornou nullptr\n", dev);
            return false;
        }

        // abrir canal (seguro em qualquer build)
        if (SL_IS_FAIL(channel->open())) {
            std::fprintf(stderr, "Falha ao abrir porta serial %s\n", dev);
            return false;
        }

        sl_result res = lidar->connect(channel);
        if (SL_IS_FAIL(res)) {
            std::fprintf(stderr, "Falha ao conectar LIDAR: %08x\n", res);
            return false;
        }

        sl_lidar_response_device_health_t health{};
        res = lidar->getHealth(health);
        if (SL_IS_FAIL(res)) {
            std::fprintf(stderr, "Falha ao obter health: %08x\n", res);
            return false;
        }
        if (health.status == SL_LIDAR_STATUS_ERROR) {
            std::fprintf(stderr, "Health ERROR: requer manutenção.\n");
            return false;
        }

        std::vector<LidarScanMode> modes;
        res = lidar->getAllSupportedScanModes(modes);
        if (SL_IS_FAIL(res) || modes.empty()) {
            std::fprintf(stderr, "Falha em getAllSupportedScanModes: %08x\n", res);
            return false;
        }
        res = lidar->startScan(false, modes[0].id);
        if (SL_IS_FAIL(res)) {
            std::fprintf(stderr, "Falha ao iniciar scan: %08x\n", res);
            return false;
        }
        return true;
    }

    void stop() { if (lidar) lidar->stop(); }

    ~LidarCtx() {
        if (lidar) { lidar->stop(); delete lidar; lidar=nullptr; }
        if (channel){ channel->close(); delete channel; channel=nullptr; }
    }
};

static inline bool obstaculo_a_frente(ILidarDriver* lidar, float limite_m=0.30f){
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    sl_result res = lidar->grabScanDataHq(nodes, count);
    if (SL_IS_FAIL(res)) return false;
    lidar->ascendScanData(nodes, count);

    for (size_t i=0;i<count;++i){
        if (nodes[i].quality==0) continue;
        float ang = nodes[i].angle_z_q14 * 90.f / 16384.f;   // graus
        if (ang < 0.f) continue;
        float dist_m = (nodes[i].dist_mm_q2/4.0f)/1000.0f;   // metros
        if (dist_m<=0.f) continue;

        bool frente_sector = (ang <= 45.f) || (ang >= 315.f);
        if (frente_sector && dist_m < limite_m) return true;
    }
    return false;
}

static void usage(const char* prog){
    std::printf(
        "Uso: sudo %s <cmd> [tempo_s]\n"
        "cmd = frente | tras | esq | dir | parar | demo | auto\n"
        "Ex.: sudo %s frente 2\n"
        "     sudo %s auto\n", prog, prog, prog
    );
}

int main(int argc, char** argv){
    std::signal(SIGINT, on_sigint);

    if (gpioInitialise() < 0){
        std::fprintf(stderr, "Falha ao inicializar pigpio (rode com sudo)\n");
        return 1;
    }
    gpioSetMode(IN1, PI_OUTPUT);
    gpioSetMode(IN2, PI_OUTPUT);
    gpioSetMode(IN3, PI_OUTPUT);
    gpioSetMode(IN4, PI_OUTPUT);
    parar();

    double t = (argc>=3)? std::atof(argv[2]) : 1.0;
    std::string cmd = (argc>=2)? argv[1] : "demo";

    auto run_for = [&](void(*fn)(), double sec){
        fn();
        std::this_thread::sleep_for(std::chrono::milliseconds(int(sec*1000)));
        parar();
    };

    if (cmd=="frente")      run_for(frente,t);
    else if (cmd=="tras")   run_for(tras,t);
    else if (cmd=="esq")    run_for(girar_esq,t);
    else if (cmd=="dir")    run_for(girar_dir,t);
    else if (cmd=="parar")  parar();
    else if (cmd=="auto"){
        std::printf("Modo autônomo: desvia de obstáculo < 0.30 m\n");
        LidarCtx ctx;
        if (!ctx.connectSerial("/dev/ttyUSB0", 460800)){
            std::fprintf(stderr, "Não foi possível inicializar o LIDAR.\n");
            gpioTerminate();
            return 2;
        }
        while (RUNNING){
            frente();
            if (obstaculo_a_frente(ctx.lidar, 0.30f)){
                parar(); std::this_thread::sleep_for(150ms);
                girar_esq(); std::this_thread::sleep_for(800ms);
                parar(); std::this_thread::sleep_for(150ms);
            }
            std::this_thread::sleep_for(50ms);
        }
        ctx.stop();
        parar();
    }
    else if (cmd=="demo"){
        std::printf("Demo: frente 1.5s, dir 1.0s, tras 1.5s, esq 1.0s\n");
        run_for(frente,1.5); std::this_thread::sleep_for(200ms);
        run_for(girar_dir,1.0); std::this_thread::sleep_for(200ms);
        run_for(tras,1.5); std::this_thread::sleep_for(200ms);
        run_for(girar_esq,1.0);
    } else {
        usage(argv[0]);
        gpioTerminate();
        return 3;
    }

    gpioTerminate();
    return 0;
}
