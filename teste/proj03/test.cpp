#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>   // termios2, BOTHER
#include <linux/serial.h>
#include <termios.h>        // <-- necessário para tcflush()
#include <cstring>

#define DEVICE "/dev/ttyUSB0"
#define BAUDRATE 256000

int main() {
    int serial_port = open(DEVICE, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_port < 0) {
        perror("Erro ao abrir porta serial");
        return 1;
    }
    std::cout << "Porta serial aberta: " << DEVICE << std::endl;

    struct termios2 tio{};
    if (ioctl(serial_port, TCGETS2, &tio) < 0) {
        perror("Erro TCGETS2");
        return 1;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = BAUDRATE;
    tio.c_ospeed = BAUDRATE;
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;

    if (ioctl(serial_port, TCSETS2, &tio) < 0) {
        perror("Erro TCSETS2");
        return 1;
    }

    std::cout << "Configuração aplicada: baud = " << BAUDRATE << "\n";

    // Limpa buffers
    tcflush(serial_port, TCIOFLUSH);

    // Comando GET_INFO: 0xA5 0x50
    unsigned char cmd[2] = {0xA5, 0x50};
    int sent = write(serial_port, cmd, 2);
    if (sent != 2) {
        perror("Erro write");
    } else {
        std::cout << "Comando GET_INFO enviado\n";
    }

    usleep(1000000); // espera 1s

    unsigned char buffer[64];
    int received = read(serial_port, buffer, sizeof(buffer));

    if (received <= 0) {
        std::cout << "Nenhum byte recebido do LIDAR." << std::endl;
    } else {
        std::cout << "Recebidos " << received << " bytes:\n";
        for (int i = 0; i < received; i++) {
            printf("%02X ", buffer[i]);
        }
        printf("\n");
    }

    close(serial_port);
    return 0;
}
