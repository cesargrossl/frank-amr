#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace std;
using namespace sl;

#define PORT 9000

ILidarDriver* drv;

bool setup_lidar(const char* port) {
    drv = *createLidarDriver();
    if (!drv) return false;
    if (IS_FAIL(drv->connect(port, 115200))) return false;

    sl_lidar_response_device_info_t info;
    if (IS_FAIL(drv->getDeviceInfo(info))) return false;
    drv->startScan(0, 1);
    return true;
}

void lidar_loop(int client_sock) {
    while (true) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);

        sl_result op_result = drv->grabScanDataHq(nodes, count);
        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            string result;

            for (size_t i = 0; i < count; i++) {
                float angle = nodes[i].angle_z_q14 * 90.f / (1 << 14);
                float dist = nodes[i].dist_mm_q2 / 4.0f / 1000.0f;
                if (dist > 0.05f && dist < 4.0f) {
                    result += to_string(angle) + "," + to_string(dist) + ";";
                }
            }

            send(client_sock, result.c_str(), result.length(), 0);
        }

        this_thread::sleep_for(chrono::milliseconds(100));
    }
}

int main() {
    const char* serial_port = "/dev/ttyUSB0";
    if (!setup_lidar(serial_port)) {
        cerr << "Erro ao conectar no LiDAR." << endl;
        return 1;
    }

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address;
    int opt = 1;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);
    bind(server_fd, (struct sockaddr*)&address, sizeof(address));
    listen(server_fd, 1);
    int addrlen = sizeof(address);

    cout << "Esperando conexão do cliente Python..." << endl;
    int client_sock = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen);
    cout << "Conectado!" << endl;

    lidar_loop(client_sock);

    drv->stop();
    drv->disconnect();
    delete drv;
    close(client_sock);
    close(server_fd);
    return 0;
}
