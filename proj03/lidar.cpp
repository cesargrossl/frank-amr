// car_lidar.cpp — Raspberry Pi + pigpio + RPLIDAR C1 (C++ puro)
// Comandos:
//   sudo ./car_lidar frente 1.5
//   sudo ./car_lidar tras 1
//   sudo ./car_lidar esq 0.8
//   sudo ./car_lidar dir 0.8
//   sudo ./car_lidar parar
//   sudo ./car_lidar auto   (andar e desviar com LIDAR)
//
// Requisitos:
//   - pigpio (sudo apt-get install pigpio)
//   - SDK Slamtec RPLIDAR (inclua rplidar.h no include path e linke a lib)

#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>
#include <csignal>
#include <vector>

#include <pigpio.h>
#include <rplidar.h>   // ajuste o include path do seu SDK

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace std::chrono_literals;
using namespace sl;

// ---------------- GPIO (BCM) p/ L298N ----------------
const int IN1 = 17; // Motor A
const int IN2 = 27; // Motor A
const int IN3 = 22; // Motor B
const int IN4 = 23; // Motor B

static volatile bool g_running = true;

static void parar()      { gpioWrite(IN1,0); gpioWrite(IN2,0); gpioWrite(IN3,0); gpioWrite(IN4,0); }
static void frente()     { gpioWrite(IN1,1); gpioWrite(IN2,0); gpioWrite(IN3,1); gpioWrite(IN4,0); }
static void tras()       { gpioWrite(IN1,0); gpioWrite(IN2,1); gpioWrite(IN3,0); gpioWrite(IN4,1); }
static void girar_esq()  { gpioWrite(IN1,1); gpioWrite(IN2,0); gpioWrite(IN3,0); gpioWrite(IN4,1); }
static void girar_dir()  { gpioWrite(IN1,0); gpioWrite(IN2,1); gpioWrite(IN3,1); gpioWrite(IN4,0); }

static void on_sigint(int) {
    g_running = false;
}

// --------------- Conexão com o LIDAR -----------------
static ILidarDriver* connect_lidar(const char* port, uint32_t baudrate) {
    IChannel* channel = createSerialPortChannel(port, baudrate);
    if (!channel) {
        std::fprintf(stderr, "Falha ao criar canal serial\n");
        return nullptr;
    }
    if (SL_IS_FAIL(channel->open())) {
        std::fprintf(stderr, "Falha ao abrir porta serial %s\n", port);
        delete channel;
        return nullptr;
    }

    ILidarDriver* lidar = createLidarDriver();
    if (!lidar) {
        std::fprintf(stderr, "Falha ao criar driver do LIDAR\n");
        delete channel;
        return nullptr;
    }

    sl_result res = lidar->connect(channel);
    if (SL_IS_FAIL(res)) {
        std::fprintf(stderr, "Falha ao conectar LIDAR: %08x\n", res);
        delete lidar;
        return nullptr;
    }

    // Health
    sl_lidar_response_device_health_t health;
    res = lidar->getHealth(health);
    if (SL_IS_FAIL(res) || health.status == SL_LIDAR_STATUS_ERROR) {
        std::fprintf(stderr, "Health LIDAR com erro (res=%08x, status=%u)\n", res, health.status);
        lidar->disconnect();
        delete lidar;
        return nullptr;
    }

    return lidar;
}

int main(int argc, char** argv) {
    if (gpioInitialise() < 0) {
        std::fprintf(stderr, "Falha ao inicializar pigpio (rode com sudo)\n");
        return 1;
    }
    std::signal(SIGINT, on_sigint);

    gpioSetMode(IN1, PI_OUTPUT);
    gpioSetMode(IN2, PI_OUTPUT);
    gpioSetMode(IN3, PI_OUTPUT);
    gpioSetMode(IN4, PI_OUTPUT);
    parar();

    double t = (argc >= 3) ? std::atof(argv[2]) : 1.0;
    std::string cmd = (argc >= 2) ? argv[1] : "demo";

    auto run_for = [&](void(*fn)(), double sec){
        fn();
        std::this_thread::sleep_for(std::chrono::milliseconds((int)(sec*1000)));
        parar();
    };

    if (cmd == "frente") run_for(frente, t);
    else if (cmd == "tras")   run_for(tras,   t);
    else if (cmd == "esq")    run_for(girar_esq, t);
    else if (cmd == "dir")    run_for(girar_dir, t);
    else if (cmd == "parar")  parar();
    else if (cmd == "auto") {
        std::puts("Modo autônomo: andando e desviando obstáculos < 0.30 m à frente");

        // *** Ajuste se necessário ***
        const char* PORT = "/dev/ttyUSB0";
        const uint32_t BAUD = 256000; // muitos C1 usam 256000; se o seu está em 460800, troque aqui

        ILidarDriver* lidar = connect_lidar(PORT, BAUD);
        if (!lidar) { gpioTerminate(); return 1; }

        // Liga motor por PWM (SDK novo não tem startMotor/stopMotor)
        lidar->setMotorPWM(660);   // 600–800 costuma ser estável

        // Usa modo típico do sensor
        LidarScanMode typical;
        sl_result res = lidar->getTypicalScanMode(typical);
        if (SL_IS_FAIL(res)) {
            std::fprintf(stderr, "Falha ao obter modo típico: %08x\n", res);
            lidar->setMotorPWM(0);
            lidar->disconnect(); delete lidar; gpioTerminate(); return 1;
        }

        res = lidar->startScan(false /*force*/, true /*useTypical*/, 0 /*options*/, &typical);
        if (SL_IS_FAIL(res)) {
            std::fprintf(stderr, "Falha ao iniciar scan: %08x\n", res);
            lidar->setMotorPWM(0);
            lidar->disconnect(); delete lidar; gpioTerminate(); return 1;
        }

        // Loop autônomo
        while (g_running) {
            frente();  // avança

            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t count = _countof(nodes);
            res = lidar->grabScanDataHq(nodes, count);
            if (SL_IS_OK(res)) {
                lidar->ascendScanData(nodes, count);

                bool obstaculo = false;
                for (size_t i = 0; i < count; ++i) {
                    if (nodes[i].quality == 0) continue;

                    float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;   // graus 0..360
                    float dist  = (nodes[i].dist_mm_q2 / 4.f) / 1000.f;    // metros

                    // Frente: 315°..360° ou 0°..45°
                    if ((angle <= 45.f || angle >= 315.f) && dist > 0.0f && dist < 0.30f) {
                        obstaculo = true;
                        break;
                    }
                }

                if (obstaculo) {
                    parar(); std::this_thread::sleep_for(200ms);
                    girar_esq(); std::this_thread::sleep_for(1000ms); // gira 1s
                    parar(); std::this_thread::sleep_for(200ms);
                }
            } else {
                std::fprintf(stderr, "Falha ao capturar dados: %08x\n", res);
                // pequena pausa para evitar loop apertado em erro
                std::this_thread::sleep_for(100ms);
            }

            std::this_thread::sleep_for(50ms);
        }

        // Encerramento
        lidar->stop();
        lidar->setMotorPWM(0);
        lidar->disconnect();
        delete lidar;
    }
    else {
        std::printf("Demo: frente 1.5s, dir 1.0s, tras 1.5s, esq 1.0s\n");
        run_for(frente, 1.5); std::this_thread::sleep_for(200ms);
        run_for(girar_dir, 1.0); std::this_thread::sleep_for(200ms);
        run_for(tras, 1.5); std::this_thread::sleep_for(200ms);
        run_for(girar_esq, 1.0);
    }

    gpioTerminate();
    return 0;
}
