#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <chrono>
#include <pigpio.h>
#include <rplidar.h>  // Inclua o header do SDK Slamtec

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace std::chrono_literals;
using namespace sl;

// Pinos BCM ligados ao L298N
const int IN1 = 17; // Motor A
const int IN2 = 27; // Motor A
const int IN3 = 22; // Motor B
const int IN4 = 23; // Motor B

static void parar()      { gpioWrite(IN1,0); gpioWrite(IN2,0); gpioWrite(IN3,0); gpioWrite(IN4,0); }
static void frente()     { gpioWrite(IN1,1); gpioWrite(IN2,0); gpioWrite(IN3,1); gpioWrite(IN4,0); }
static void tras()       { gpioWrite(IN1,0); gpioWrite(IN2,1); gpioWrite(IN3,0); gpioWrite(IN4,1); }
static void girar_esq()  { gpioWrite(IN1,1); gpioWrite(IN2,0); gpioWrite(IN3,0); gpioWrite(IN4,1); }
static void girar_dir()  { gpioWrite(IN1,0); gpioWrite(IN2,1); gpioWrite(IN3,1); gpioWrite(IN4,0); }

int main(int argc, char** argv) {
    if (gpioInitialise() < 0) { // precisa rodar com sudo
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
        std::printf("Modo autônomo: Andando e evitando obstáculos a 30cm\n");

        // Inicializa o canal serial para o RPLIDAR C1
        IChannel* channel = *createSerialPortChannel("/dev/ttyUSB0", 460800);
        if (SL_IS_FAIL((channel)->open())) {
            fprintf(stderr, "Falha ao abrir porta serial\n");
            delete channel;
            gpioTerminate();
            return 1;
        }

        // Cria o driver do LIDAR
        ILidarDriver* lidar = *createLidarDriver();
        sl_result res = lidar->connect(channel);
        if (SL_IS_FAIL(res)) {
            fprintf(stderr, "Falha ao conectar LIDAR %08x\n", res);
            delete lidar;
            delete channel;
            gpioTerminate();
            return 1;
        }

        // Verifica saúde do LIDAR
        sl_lidar_response_device_health_t healthinfo;
        res = lidar->getHealth(healthinfo);
        if (SL_IS_OK(res)) {
            if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
                fprintf(stderr, "LIDAR com erro de saúde!\n");
                delete lidar;
                delete channel;
                gpioTerminate();
                return 1;
            }
        } else {
            fprintf(stderr, "Falha ao obter health %08x\n", res);
            delete lidar;
            delete channel;
            gpioTerminate();
            return 1;
        }

        // Inicia o motor e o scan
        lidar->startMotor();
        std::vector<LidarScanMode> scanModes;
        lidar->getAllSupportedScanModes(scanModes);
        res = lidar->startScan(false, scanModes[0].id);  // Modo padrão para C1
        if (SL_IS_FAIL(res)) {
            fprintf(stderr, "Falha ao iniciar scan %08x\n", res);
            lidar->stopMotor();
            delete lidar;
            delete channel;
            gpioTerminate();
            return 1;
        }

        // Loop autônomo
        while (true) {
            frente();  // Move para frente

            // Captura dados do scan
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t count = _countof(nodes);
            res = lidar->grabScanDataHq(nodes, count);
            if (SL_IS_OK(res)) {
                lidar->ascendScanData(nodes, count);  // Ordena por ângulo

                bool obstaculo = false;
                for (size_t i = 0; i < count; ++i) {
                    if (nodes[i].quality != 0) {  // Ponto válido
                        float angle = nodes[i].angle_z_q14 * 90.f / 16384.f;
                        float dist = nodes[i].dist_mm_q2 / 4.f / 1000.f;

                        // Verifica frente: ângulos de 315° a 45° (equivalente a -45° a 45°)
                        if ((angle <= 45.f || angle >= 315.f) && dist < 0.3f && dist > 0.0f) {
                            obstaculo = true;
                            break;
                        }
                    }
                }

                if (obstaculo) {
                    parar();
                    std::this_thread::sleep_for(200ms);
                    girar_esq();  // Gira para esquerda
                    std::this_thread::sleep_for(1000ms);  // 1s de giro
                    parar();
                    std::this_thread::sleep_for(200ms);
                }
            } else {
                printf("Falha ao capturar dados %08x\n", res);
            }

            // Pequeno delay para evitar loop apertado
            std::this_thread::sleep_for(50ms);
        }

        // Cleanup (nunca alcançado sem sinal, mas para referência)
        lidar->stop();
        lidar->stopMotor();
        delete lidar;
        delete channel;
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