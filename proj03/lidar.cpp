#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cstdint>

#define LIDAR_SERIAL "/dev/ttyUSB0"
#define LIDAR_BAUD 256000

int abrir_serial(const char* device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Erro ao abrir serial");
        return -1;
    }

    struct termios2 tio2;
    if (ioctl(fd, TCGETS2, &tio2) < 0) {
        perror("Erro ao obter termios2");
        close(fd);
        return -1;
    }

    tio2.c_cflag &= ~CBAUD;
    tio2.c_cflag |= BOTHER;
    tio2.c_ispeed = baudrate;
    tio2.c_ospeed = baudrate;

    tio2.c_cflag |= (CLOCAL | CREAD);
    tio2.c_cflag &= ~CSIZE;
    tio2.c_cflag |= CS8;
    tio2.c_cflag &= ~PARENB;
    tio2.c_cflag &= ~CSTOPB;
    tio2.c_cflag &= ~CRTSCTS;

    tio2.c_iflag = IGNPAR;
    tio2.c_oflag = 0;
    tio2.c_lflag = 0;

    tio2.c_cc[VTIME] = 1;
    tio2.c_cc[VMIN] = 0;

    if (ioctl(fd, TCSETS2, &tio2) < 0) {
        perror("Erro ao aplicar termios2");
        close(fd);
        return -1;
    }

    return fd;
}

void enviar_comando(int fd, uint8_t cmd1, uint8_t cmd2) {
    uint8_t comando[2] = {cmd1, cmd2};
    write(fd, comando, 2);
}

void ler_resposta(int fd, int bytes) {
    uint8_t buffer[128] = {0};
    int lidos = read(fd, buffer, bytes);
    std::cout << "Bytes lidos: " << lidos << std::endl;

    for (int i = 0; i < lidos; ++i)
        printf("%02X ", buffer[i]);

    std::cout << std::endl;
}

int main() {
    int serial_fd = abrir_serial(LIDAR_SERIAL, LIDAR_BAUD);
    if (serial_fd < 0) {
        return 1;
    }

    // Comando GET_INFO: 0xA5 0x50
    enviar_comando(serial_fd, 0xA5, 0x50);
    usleep(50000); // Aguarda resposta

    ler_resposta(serial_fd, 32);

    close(serial_fd);
    return 0;
}
