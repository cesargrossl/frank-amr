#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/serial.h>     // struct serial_struct
#include <sys/types.h>
#include <sys/stat.h>

#define LIDAR_SERIAL "/dev/ttyUSB0"
#define BAUDRATE 256000

int configurar_serial(int fd, int baudrate) {
    struct serial_struct ser;
    if (ioctl(fd, TIOCGSERIAL, &ser) < 0) {
        perror("Erro ao obter configuração serial");
        return -1;
    }

    ser.flags &= ~ASYNC_SPD_MASK;
    ser.flags |= ASYNC_SPD_CUST;
    ser.custom_divisor = ser.baud_base / baudrate;

    if (ioctl(fd, TIOCSSERIAL, &ser) < 0) {
        perror("Erro ao configurar baudrate customizado");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfmakeraw(&options);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    tcsetattr(fd, TCSANOW, &options);
    return 0;
}

int abrir_serial(const char* device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Erro ao abrir porta serial");
        return -1;
    }

    if (configurar_serial(fd, baudrate) < 0) {
        close(fd);
        return -1;
    }

    return fd;
}

void enviar_comando(int fd, uint8_t cmd1, uint8_t cmd2) {
    uint8_t cmd[] = {cmd1, cmd2};
    write(fd, cmd, 2);
}

void ler_resposta(int fd, int tamanho) {
    uint8_t buffer[128] = {0};
    int lidos = read(fd, buffer, tamanho);
    std::cout << "Lidos: " << lidos << " bytes: ";
    for (int i = 0; i < lidos; ++i)
        printf("%02X ", buffer[i]);
    std::cout << std::endl;
}

int main() {
    int fd = abrir_serial(LIDAR_SERIAL, BAUDRATE);
    if (fd < 0) return 1;

    enviar_comando(fd, 0xA5, 0x50); // GET_INFO
    usleep(50000);
    ler_resposta(fd, 32);
    close(fd);
    return 0;
}
