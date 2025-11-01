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

// libgpiod v2 handles/objects
static gpiod_chip*           chip   = nullptr;
static gpiod_line_settings*  st     = nullptr;
static gpiod_line_config*    lcfg   = nullptr;
static gpiod_request_config* rcfg   = nullptr;
static gpiod_line_request*   req    = nullptr;

// buffer de valores para set múltiplo
static gpiod_line_value vals[4] = {
  GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE,
  GPIOD_LINE_VALUE_INACTIVE, GPIOD_LINE_VALUE_INACTIVE
};
static unsigned int offsets[4] = {IN1, IN2, IN3, IN4};

static bool gpio_init(const char* chip_path="/dev/gpiochip0") {
  chip = gpiod_chip_open(chip_path);
  if (!chip) {
    std::fprintf(stderr, "Erro: não abriu %s (use sudo ou entre no grupo gpio)\n", chip_path);
    return false;
  }

  st = gpiod_line_settings_new();
  if (!st) { std::fprintf(stderr, "gpiod_line_settings_new\n"); return false; }
  gpiod_line_settings_set_direction(st, GPIOD_LINE_DIRECTION_OUTPUT);
  gpiod_line_settings_set_output_value(st, GPIOD_LINE_VALUE_INACTIVE);

  lcfg = gpiod_line_config_new();
  if (!lcfg) { std::fprintf(stderr, "gpiod_line_config_new\n"); return false; }
  if (gpiod_line_config_add_line_settings(lcfg, offsets, 4, st) < 0) {
    std::fprintf(stderr, "gpiod_line_config_add_line_settings\n"); return false;
  }

  rcfg = gpiod_request_config_new();
  if (!rcfg) { std::fprintf(stderr, "gpiod_request_config_new\n"); return false; }
  gpiod_request_config_set_consumer(rcfg, "amr-motors");

  req = gpiod_chip_request_lines(chip, rcfg, lcfg);
  if (!req) {
    std::fprintf(stderr, "gpiod_chip_request_lines (pinos ocupados?)\n");
    return false;
  }

  // garante todos OFF
  gpiod_line_request_set_values(req, vals);
  return true;
}

static void gpio_close() {
  if (req)  { gpiod_line_request_release(req); req=nullptr; }
  if (rcfg) { gpiod_request_config_free(rcfg); rcfg=nullptr; }
  if (lcfg) { gpiod_line_config_free(lcfg);    lcfg=nullptr; }
  if (st)   { gpiod_line_settings_free(st);    st=nullptr; }
  if (chip) { gpiod_chip_close(chip);          chip=nullptr; }
}

// ---- Controle motores (L298N) ----
static inline void set_raw(bool a1,bool a2,bool b1,bool b2) {
  vals[0] = a1 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE; // IN1
  vals[1] = a2 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE; // IN2
  vals[2] = b1 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE; // IN3
  vals[3] = b2 ? GPIOD_LINE_VALUE_ACTIVE   : GPIOD_LINE_VALUE_INACTIVE; // IN4
  if (gpiod_line_request_set_values(req, vals) < 0) {
    std::fprintf(stderr, "Aviso: falha ao setar GPIO (ocupado?)\n");
  }
}
static inline void parar()     { set_raw(false,false,false,false); }
static inline void frente()    { set_raw(true ,false, true ,false); }
static inline void tras()      { set_raw(false,true ,false,true ); }
static inline void girar_esq() { set_raw(true ,false, false,true ); }
static inline void girar_dir() { set_raw(false,true , true ,false); }

// ---- LIDAR ----
struct LidarCtx {
  ILidarDriver* lidar{nullptr};
  IChannel*     channel{nullptr};

  bool connectSerial(const char* dev="/dev/ttyUSB0", uint32_t baud=460800) {
    lidar = *createLidarDriver();
    if (!lidar) { std::fprintf(stderr, "createLidarDriver nullptr\n"); return false; }

    auto ch_res = createSerialPortChannel(dev, baud); // Result<IChannel*>
    sl_result rc = ch_res;                            // usa operator sl_result()
    if (SL_IS_FAIL(rc)) {
      std::fprintf(stderr, "createSerialPortChannel(%s) falhou: %08x\n", dev, rc);
      return false;
    }
    channel = ch_res.value;                           // pega o ponteiro (sem ())
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

  std::string cmd = (argc>=2)? argv[1] : "demo";
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
      std::fprintf(stderr, "LIDAR não inicializou.\n");
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


sudo apt-get update
sudo apt-get install -y g++ libgpiod-dev

g++ -std=c++17 motor_lidar_gpiod.cpp -o motor_lidar \
  -I../rplidar_sdk/sdk/include \
  -L../rplidar_sdk/output/Linux/Release \
  -lsl_lidar_sdk -lgpiod -lpthread -lrt
