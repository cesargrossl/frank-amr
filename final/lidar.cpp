#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <csignal>
#include <vector>
#include <string>

#include <gpiod.h>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace std::chrono_literals;
using namespace sl;

// GPIO offsets BCM no gpiochip0
static const unsigned int IN1 = 17;
static const unsigned int IN2 = 27;
static const unsigned int IN3 = 22;
static const unsigned int IN4 = 23;

static gpiod_chip *chip;
static gpiod_line_bulk lines;
static gpiod_line_settings *settings;
static gpiod_line_config *config;
static gpiod_request_config *req_cfg;
static gpiod_line_request *req;

static volatile bool RUNNING = true;
void on_sigint(int) { RUNNING = false; }

// ------- Motores L298N usando libgpiod ---------
static void gpio_write(bool a1,bool a2,bool b1,bool b2){
    gpiod_line_value v[4] = {
        a1 ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE,
        a2 ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE,
        b1 ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE,
        b2 ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE
    };
    gpiod_line_request_set_values(req, v);
}

static inline void parar()     { gpio_write(0,0,0,0); }
static inline void frente()    { gpio_write(1,0,1,0); }
static inline void tras()      { gpio_write(0,1,0,1); }
static inline void girar_esq() { gpio_write(1,0,0,1); }
static inline void girar_dir() { gpio_write(0,1,1,0); }

// ------- LIDAR -------
struct LidarCtx {
    ILidarDriver* lidar = nullptr;
    IChannel* channel = nullptr;

    bool connectSerial(const char* dev="/dev/ttyUSB0", uint32_t baud=460800){
        lidar = *createLidarDriver();
        if (!lidar) return false;

        auto ch_res = createSerialPortChannel(dev, baud);
        sl_result rc = ch_res;
        if (SL_IS_FAIL(rc)) return false;

        channel = ch_res.value;
        if (!channel) return false;

        sl_result res = lidar->connect(channel);
        if (SL_IS_FAIL(res)) return false;

        sl_lidar_response_device_health_t health{};
        res = lidar->getHealth(health);
        if (SL_IS_FAIL(res) || health.status == SL_LIDAR_STATUS_ERROR) return false;

        std::vector<LidarScanMode> modes;
        lidar->getAllSupportedScanModes(modes);
        lidar->startScan(false, modes[0].id);

        return true;
    }

    ~LidarCtx() {
        if (lidar) { lidar->stop(); delete lidar; }
        if (channel) delete channel;
    }
};

bool obstaculo(ILidarDriver* lidar, float limite=0.30){
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = 8192;

    if (SL_IS_FAIL(lidar->grabScanDataHq(nodes, count))) return false;
    lidar->ascendScanData(nodes, count);

    for (size_t i=0;i<count;i++){
        if (nodes[i].quality == 0) continue;
        float ang = nodes[i].angle_z_q14 * 90.f / 16384.f;
        float dist = (nodes[i].dist_mm_q2 / 4.0f) / 1000.0f;

        if ((ang <= 45.f || ang >= 315.f) && dist < limite)
            return true;
    }
    return false;
}

// ------- Main -------
int main(int argc, char** argv){
    std::signal(SIGINT, on_sigint);

    // Init gpiod
    chip = gpiod_chip_open("/dev/gpiochip0");

    unsigned int offsets[4] = {IN1,IN2,IN3,IN4};
    settings = gpiod_line_settings_new();
    gpiod_line_settings_set_direction(settings,GPIOD_LINE_DIRECTION_OUTPUT);

    config = gpiod_line_config_new();
    gpiod_line_config_add_line_settings(config, offsets, 4, settings);

    req_cfg = gpiod_request_config_new();
    gpiod_request_config_set_consumer(req_cfg, "AMR-MOTORS");

    req = gpiod_chip_request_lines(chip, req_cfg, config);
    parar();

    std::string cmd = (argc>=2)? argv[1] : "auto";
    double t = (argc>=3)? atof(argv[2]) : 1.0;

    if(cmd=="frente"){ frente(); std::this_thread::sleep_for(t*1s); parar(); return 0; }
    if(cmd=="tras"){ tras(); std::this_thread::sleep_for(t*1s); parar(); return 0; }

    if(cmd=="auto"){
        LidarCtx ctx;
        if(!ctx.connectSerial("/dev/ttyUSB0",460800)){
            printf("Lidar não inicializou\n");
            return 0;
        }

        while(RUNNING){
            frente();
            if(obstaculo(ctx.lidar)){
                parar();
                girar_esq();
                std::this_thread::sleep_for(800ms);
                parar();
            }
            std::this_thread::sleep_for(50ms);
        }
    }

    parar();
    return 0;
}



g++ -std=c++17 motor_lidar_gpiod.cpp -o motor_lidar \
  -I../rplidar_sdk/sdk/include \
  -L../rplidar_sdk/output/Linux/Release \
  -lsl_lidar_sdk -pthread -lrt -lgpiod
