#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/termios.h> // Para struct termios2
#include <asm/ioctls.h>    // Para TCGETS2 / TCSETS2
#include <asm/termbits.h>  // Para BOTHER

#define LIDAR_SERIAL "/dev/ttyUSB0"
#define BAUDRATE 256000

int abrir_serial(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Erro ao abrir porta serial");
        return -1;
    }

    struct termios2 tio;
    if (ioctl(fd, TCGETS2, &tio) < 0) {
        perror("Erro TCGETS2");
        close(fd);
        return -1;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = BAUDRATE;
    tio.c_ospeed = BAUDRATE;
    tio.c_cflag |= (CLOCAL | CREAD | CS8);
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    tio.c_cc[VTIME] = 1;
    tio.c_cc[VMIN] = 0;

    if (ioctl(fd, TCSETS2, &tio) < 0) {
        perror("Erro TCSETS2");
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
    int fd = abrir_serial(LIDAR_SERIAL);
    if (fd < 0) return 1;

    enviar_comando(fd, 0xA5, 0x50); // GET_INFO
    usleep(50000);
    ler_resposta(fd, 32);
    close(fd);
    return 0;
}
