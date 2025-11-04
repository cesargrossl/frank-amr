#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <pigpio.h>
#include <rplidar.h>
#include <chrono>
#include <thread>
#include <functional>
#include <csignal>

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace std::chrono_literals;
using namespace sl;

// Variável global para controle
volatile sig_atomic_t running = 1;

void signal_handler(int signum) {
    running = 0;
}

// Pinos BCM ligados ao L298N
const int IN1 = 17; // Motor A (Pino 11)
const int IN2 = 27; // Motor A (Pino 13)
const int IN3 = 22; // Motor B (Pino 15)
const int IN4 = 23; // Motor B (Pino 16)

static void parar()      { gpioWrite(IN1, 0); gpioWrite(IN2, 0); gpioWrite(IN3, 0); gpioWrite(IN4, 0); }
static void frente()     { gpioWrite(IN1, 1); gpioWrite(IN2, 0); gpioWrite(IN3, 1); gpioWrite(IN4, 0); }
static void tras()       { gpioWrite(IN1, 0); gpioWrite(IN2, 1); gpioWrite(IN3, 0); gpioWrite(IN4, 1); }
static void girar_esq()  { gpioWrite(IN1, 1); gpioWrite(IN2, 0); gpioWrite(IN3, 0); gpioWrite(IN4, 1); }
static void girar_dir()  { gpioWrite(IN1, 0); gpioWrite(IN2, 1); gpioWrite(IN3, 1); gpioWrite(IN4, 0); }

class CarrinhoNode : public rclcpp::Node {
public:
    CarrinhoNode() : Node("carrinho_node"), lidar(nullptr), channel(nullptr) {
        // Subscrever a /cmd_vel para controle de motores
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CarrinhoNode::cmd_vel_callback, this, std::placeholders::_1));

        // Publicar /scan para LIDAR (se não usar pacote externo)
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

        // Inicializar pigpio
        if (gpioInitialise() < 0) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao inicializar pigpio");
            return;
        }

        gpioSetMode(IN1, PI_OUTPUT);
        gpioSetMode(IN2, PI_OUTPUT);
        gpioSetMode(IN3, PI_OUTPUT);
        gpioSetMode(IN4, PI_OUTPUT);
        parar();

        // Inicializar LIDAR
        initialize_lidar();

        // Timer para publicar scan periodicamente
        timer_ = this->create_wall_timer(50ms, std::bind(&CarrinhoNode::publish_scan, this));
    }

    ~CarrinhoNode() {
        parar();
        if (lidar) {
            lidar->stop();
            delete lidar;
        }
        if (channel) delete channel;
        gpioTerminate();
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    ILidarDriver* lidar;
    IChannel* channel;

    void initialize_lidar() {
        channel = *createSerialPortChannel("/dev/ttyUSB0", 460800);
        if (SL_IS_FAIL((channel)->open())) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao abrir porta serial");
            return;
        }

        lidar = *createLidarDriver();
        sl_result res = lidar->connect(channel);
        if (SL_IS_FAIL(res)) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao conectar LIDAR %08x", res);
            return;
        }

        sl_lidar_response_device_health_t healthinfo;
        res = lidar->getHealth(healthinfo);
        if (SL_IS_FAIL(res) || healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            RCLCPP_ERROR(this->get_logger(), "LIDAR com erro de saúde!");
            return;
        }

        std::vector<LidarScanMode> scanModes;
        lidar->getAllSupportedScanModes(scanModes);
        res = lidar->startScan(false, scanModes[0].id);
        if (SL_IS_FAIL(res)) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao iniciar scan %08x", res);
            return;
        }
    }

    void publish_scan() {
        if (!lidar) return;

        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);
        sl_result res = lidar->grabScanDataHq(nodes, count);
        if (SL_IS_OK(res)) {
            lidar->ascendScanData(nodes, count);

            sensor_msgs::msg::LaserScan scan_msg;
            scan_msg.header.stamp = this->now();
            scan_msg.header.frame_id = "laser";
            scan_msg.angle_min = 0.0;
            scan_msg.angle_max = 2 * M_PI;
            scan_msg.angle_increment = (2 * M_PI) / count;
            scan_msg.range_min = 0.05;
            scan_msg.range_max = 12.0;
            scan_msg.ranges.resize(count);
            scan_msg.intensities.resize(count);

            for (size_t i = 0; i < count; ++i) {
                scan_msg.ranges[i] = nodes[i].dist_mm_q2 / 4.0f / 1000.0f;
                scan_msg.intensities[i] = nodes[i].quality;
            }

            scan_pub_->publish(scan_msg);

            // Lógica autônoma (desvio) se não houver cmd_vel externo
            bool obstaculo = false;
            for (size_t i = 0; i < count; ++i) {
                if (nodes[i].quality != 0) {
                    float angle = nodes[i].angle_z_q14 * 90.f / 16384.f * M_PI / 180.0f;
                    float dist = nodes[i].dist_mm_q2 / 4.0f / 1000.0f;
                    if ((angle < M_PI / 4 || angle > 7 * M_PI / 4) && dist < 0.3f && dist > 0.0f) {
                        obstaculo = true;
                        break;
                    }
                }
            }

            if (obstaculo) {
                parar();
                std::this_thread::sleep_for(200ms);
                girar_esq();
                std::this_thread::sleep_for(1000ms);
                parar();
                std::this_thread::sleep_for(200ms);
                frente();
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Falha ao capturar dados %08x", res);
        }
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Controle baseado em velocity (de Nav2 ou teleop)
        double linear = msg->linear.x;
        double angular = msg->angular.z;

        if (linear > 0) {
            frente();
        } else if (linear < 0) {
            tras();
        } else if (angular > 0) {
            girar_esq();
        } else if (angular < 0) {
            girar_dir();
        } else {
            parar();
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarrinhoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}