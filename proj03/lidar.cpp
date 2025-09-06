// Controle de motores com L298N e desvio de obstáculos usando RPLIDAR C1
// Sem ROS, apenas C++ puro para Raspberry Pi

#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <vector>
#include <cmath>

#define IN1_GPIO "/sys/class/gpio/gpio17/value"  // GPIO17 -> IN1
#define IN2_GPIO "/sys/class/gpio/gpio27/value"  // GPIO27 -> IN2
#define IN3_GPIO "/sys/class/gpio/gpio22/value"  // GPIO22 -> IN3
#define IN4_GPIO "/sys/class/gpio/gpio23/value"  // GPIO23 -> IN4
#define LIDAR_SERIAL "/dev/ttyUSB0"

void write_gpio(const std::string &path, const std::string &value) {
    std::ofstream gpio(path);
    if (gpio.is_open()) {
        gpio << value;
        gpio.close();
    }
}

void frente() {
    write_gpio(IN1_GPIO, "1");
    write_gpio(IN2_GPIO, "0");
    write_gpio(IN3_GPIO, "1");
    write_gpio(IN4_GPIO, "0");
}

void parar() {
    write_gpio(IN1_GPIO, "0");
    write_gpio(IN2_GPIO, "0");
    write_gpio(IN3_GPIO, "0");
    write_gpio(IN4_GPIO, "0");
}

void girar_direita() {
    write_gpio(IN1_GPIO, "1");
    write_gpio(IN2_GPIO, "0");
    write_gpio(IN3_GPIO, "0");
    write_gpio(IN4_GPIO, "1");
}

int abrir_serial(const char* device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;

    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

bool detectar_obstaculo(int serial_fd) {
    // Comando Start Scan: 0xA5 0x20
    unsigned char start_scan[] = {0xA5, 0x20};
    write(serial_fd, start_scan, 2);
    usleep(200000);

    unsigned char buffer[5];
    read(serial_fd, buffer, 5);  // resposta de 5 bytes

    unsigned char sample[47];
    int n = read(serial_fd, sample, 47);
    if (n < 5) return false;

    for (int i = 0; i < n - 4; i += 5) {
        int distance = ((sample[i+3] << 8) | sample[i+2]) / 4;  // mm
        float angle = ((sample[i+1] >> 1) + ((sample[i+2] & 0x01) << 7)) * 0.25;
        if (distance > 100 && distance < 300 && angle > 60 && angle < 120) {
            return true;  // obstáculo entre 60° e 120° a menos de 30cm
        }
    }
    return false;
}

int main() {
    int serial_fd = abrir_serial(LIDAR_SERIAL, B256000);
    if (serial_fd < 0) {
        std::cerr << "Erro ao abrir LIDAR" << std::endl;
        return 1;
    }

    while (true) {
        if (detectar_obstaculo(serial_fd)) {
            parar();
            usleep(500000);
            girar_direita();
            usleep(700000);
            parar();
        } else {
            frente();
        }
        usleep(100000);
    }

    close(serial_fd);
    return 0;
}
