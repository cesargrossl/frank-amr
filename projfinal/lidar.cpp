#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <cstring>

#include <gpiod.h>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace std::chrono_literals;
using namespace sl;

// ------------------------------
// GPIO (BCM offsets) - libgpiod v1
static unsigned int IN1 = 17; // Motor A
static unsigned int IN2 = 27; // Motor A
static unsigned int IN3 = 22; // Motor B
static unsigned int IN4 = 23; // Motor B

// Inversão por software (padrão ligado p/ seu hardware)
static bool INV_A = true;
static bool INV_B = true;

// STBY opcional (TB6612FNG). Se <0, não usa.
static int STBY = -1;

// Chip path
static std::string CHIP_PATH = "/dev/gpiochip0";

// LIDAR
static std::string LIDAR_PORT = "/dev/ttyUSB0";
static uint32_t    LIDAR_BAUD = 460800;

// ------------------------------
// libgpiod v1 handles
static gpiod_chip* chip   = nullptr;
static gpiod_line* l_in1  = nullptr;
static gpiod_line* l_in2  = nullptr;
static gpiod_line* l_in3  = nullptr;
static gpiod_line* l_in4  = nullptr;
static gpiod_line* l_stby = nullptr;

// ------------------------------
// Parse simples de flags (--k=v)
static void parse_args(int argc, char** argv, std::string& cmd, double& t) {
  cmd = (argc >= 2) ? argv[1] : "auto";
  t   = (argc >= 3) ? std::atof(argv[2]) : 1.0;

  for (int i = 3; i < argc; ++i) {
    const char* a = argv[i];
    if (std::strncmp(a, "--chip=", 7) == 0) {
      CHIP_PATH = std::string(a+7);
    } else if (std::strncmp(a, "--invA=", 7) == 0) {
      INV_A = std::atoi(a+7) != 0;
    } else if (std::strncmp(a, "--invB=", 7) == 0) {
      INV_B = std::atoi(a+7) != 0;
    } else if (std::strncmp(a, "--stby=", 7) == 0) {
      STBY = std::atoi(a+7);
    } else if (std::strncmp(a, "--port=", 7) == 0) {
      LIDAR_PORT = std::string(a+7);
    } else if (std::strncmp(a, "--baud=", 7) == 0) {
      LIDAR_BAUD = (uint32_t)std::strtoul(a+7, nullptr, 10);
    }
  }
}

static void usage(const char* prog){
  std::printf(
    "Uso: %s <cmd> [tempo_s] [opções]\n"
    "cmd = frente | tras | esq | dir | parar | demo | auto\n"
    "Opções:\n"
    "  --chip=/dev/gpiochipN   (padrão: /dev/gpiochip0)\n"
    "  --invA=0|1              (padrão: 1)\n"
    "  --invB=0|1              (padrão: 1)\n"
    "  --stby=BCM              (pino STBY do TB6612FNG; se ausente, não usa)\n"
    "  --port=/dev/ttyUSB0     (porta serial do LIDAR; auto usa)\n"
    "  --baud=460800           (baud do LIDAR)\n"
    "Ex.: sudo %s auto --stby=24 --invA=1 --invB=1\n",
    prog, prog
  );
}

// ------------------------------
// GPIO init/close (libgpiod v1)
static bool gpio_init() {
  chip = gpiod_chip_open(CHIP_PATH.c_str());
  if (!chip) {
    std::fprintf(stderr, "Erro: não abriu %s (sudo/grupo gpio?)\n", CHIP_PATH.c_str());
    return false;
  }

  l_in1 = gpiod_chip_get_line(chip, IN1);
  l_in2 = gpiod_chip_get_line(chip, IN2);
  l_in3 = gpiod_chip_get_line(chip, IN3);
  l_in4 = gpiod_chip_get_line(chip, IN4);
  if (!l_in1 || !l_in2 || !l_in3 || !l_in4) {
    std::fprintf(stderr, "Erro: falha ao obter linhas IN1..IN4\n");
    return false;
  }

  if (gpiod_line_request_output(l_in1, "amr-motors", 0) < 0 ||
      gpiod_line_request_output(l_in2, "amr-motors", 0) < 0 ||
      gpiod_line_request_output(l_in3, "amr-motors", 0) < 0 ||
      gpiod_line_request_output(l_in4, "amr-motors", 0) < 0) {
    std::fprintf(stderr, "Erro: falha ao solicitar linhas como saída\n");
    return false;
  }

  // STBY opcional
  if (STBY >= 0) {
    l_stby = gpiod_chip_get_line(chip, (unsigned)STBY);
    if (!l_stby) {
      std::fprintf(stderr, "Aviso: falha ao obter STBY (BCM%d)\n", STBY);
    } else if (gpiod_line_request_output(l_stby, "amr-motors", 1) < 0) {
      std::fprintf(stderr, "Aviso: falha ao solicitar STBY como saída\n");
      l_stby = nullptr;
    } else {
      gpiod_line_set_value(l_stby, 1); // habilita driver
    }
  }

  // tudo em 0
  gpiod_line_set_value(l_in1, 0);
  gpiod_line_set_value(l_in2, 0);
  gpiod_line_set_value(l_in3, 0);
  gpiod_line_set_value(l_in4, 0);
  return true;
}

static void gpio_close() {
  // zera
  auto safe_set = [](gpiod_line* l){ if (l) gpiod_line_set_value(l,0); };
  safe_set(l_in1); safe_set(l_in2); safe_set(l_in3); safe_set(l_in4);
  if (l_stby) gpiod_line_set_value(l_stby, 0);

  auto rel = [](gpiod_line* l){ if (l) gpiod_line_release(l); };
  rel(l_in1); rel(l_in2); rel(l_in3); rel(l_in4); rel(l_stby);

  if (chip) gpiod_chip_close(chip);
  chip = nullptr; l_in1=l_in2=l_in3=l_in4=l_stby=nullptr;
}

// ------------------------------
// Motor helpers (v1)
static inline void set_raw(bool a1,bool a2,bool b1,bool b2) {
  if (gpiod_line_set_value(l_in1, a1?1:0) < 0 ||
      gpiod_line_set_value(l_in2, a2?1:0) < 0 ||
      gpiod_line_set_value(l_in3, b1?1:0) < 0 ||
      gpiod_line_set_value(l_in4, b2?1:0) < 0) {
    std::fprintf(stderr, "Aviso: falha ao setar GPIO (ocupado?)\n");
  }
}

static inline void set(bool a1,bool a2,bool b1,bool b2){
  if (INV_A) std::swap(a1, a2);
  if (INV_B) std::swap(b1, b2);
  set_raw(a1,a2,b1,b2);
}

static inline void parar()     { set(false,false,false,false); }
static inline void frente()    { set(true ,false, true ,false); }  // frente (com INV corrige seu HW)
static inline void tras()      { set(false,true ,false,true ); }
static inline void girar_esq() { set(true ,false, false,true ); }  // A fwd, B rev
static inline void girar_dir() { set(false,true , true ,false); }  // A rev, B fwd

// ------------------------------
// LIDAR
struct LidarCtx {
  ILidarDriver* lidar{nullptr};
  IChannel*     channel{nullptr};

  bool connectSerial(const char* dev, uint32_t baud) {
    lidar = *createLidarDriver();
    if (!lidar) { std::fprintf(stderr, "createLidarDriver nullptr\n"); return false; }

    // Result<IChannel*>
    auto ch_res = createSerialPortChannel(dev, baud);
    sl_result rc = ch_res; // usa operator sl_result() p/ checar sucesso
    if (SL_IS_FAIL(rc)) {
      std::fprintf(stderr, "createSerialPortChannel(%s) falhou: %08x\n", dev, rc);
      return false;
    }

    channel = ch_res.value; // pega ponteiro do canal
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

// Obstáculo: frente em [315°,360°) ∪ [0°,45°], dist < limite
static inline bool obstaculo_a_frente(ILidarDriver* lidar, float limite_m=0.30f) {
  sl_lidar_response_measurement_node_hq_t nodes[8192];
  size_t count = sizeof(nodes)/sizeof(nodes[0]);

  sl_result r = lidar->grabScanDataHq(nodes, count);
  if (SL_IS_FAIL(r)) return false;
  lidar->ascendScanData(nodes, count);

  for (size_t i=0;i<count;++i) {
    if (nodes[i].quality == 0) continue;
    float ang   = nodes[i].angle_z_q14 * 90.f / 16384.f;   // graus (0..360)
    float distm = (nodes[i].dist_mm_q2 / 4.0f) / 1000.0f;  // metros
    if (distm <= 0.f) continue;
    bool frente = (ang <= 45.f) || (ang >= 315.f);
    if (frente && distm < limite_m) return true;
  }
  return false;
}

// ------------------------------
static volatile bool RUNNING = true;
static void on_sigint(int){ RUNNING=false; }

// ------------------------------
int main(int argc, char** argv) {
  std::signal(SIGINT, on_sigint);

  std::string cmd; double t=1.0;
  parse_args(argc, argv, cmd, t);

  if (!gpio_init()) return 1;

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
    std::printf("Auto: evitar obstáculo < 0.30 m na frente (porta=%s, baud=%u)\n",
                LIDAR_PORT.c_str(), LIDAR_BAUD);
    LidarCtx ctx;
    if (!ctx.connectSerial(LIDAR_PORT.c_str(), LIDAR_BAUD)) {
      std::fprintf(stderr, "LIDAR não inicializou.\n");
      gpio_close();
      return 2;
    }
    while (RUNNING) {
      frente();
      if (obstaculo_a_frente(ctx.lidar, 0.30f)) {
        parar(); std::this_thread::sleep_for(150ms);
        // pequena manobra: giro à esquerda por 800ms
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


/*sudo apt update
sudo apt install -y g++ libgpiod-dev

# Ajuste os caminhos do SDK (INCLUDE/LIB) conforme a sua pasta
g++ -std=c++17 lidar.cpp -o lidar \
  -I../rplidar_sdk/sdk/include \
  -L../rplidar_sdk/output/Linux/Release \
  -lsl_lidar_sdk -lgpiod -pthread -O2
// sudo ./lidar auto

*/