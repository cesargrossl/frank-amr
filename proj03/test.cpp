#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

#define DEVICE "/dev/ttyUSB0"
#define BAUDRATE 256000

int main() {
    int serial_port = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port < 0) {
        std::cerr << "Erro ao abrir " << DEVICE << std::endl;
        return 1;
    }
    std::cout << "Porta serial aberta: " << DEVICE << std::endl;

    struct termios2 tio;
    if (ioctl(serial_port, TCGETS2, &tio) < 0) {
        std::cerr << "Erro ao obter configuração termios2\n";
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
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;

    if (ioctl(serial_port, TCSETS2, &tio) < 0) {
        std::cerr << "Erro ao aplicar configuração termios2\n";
        return 1;
    }

    std::cout << "Configuração da porta serial aplicada (baud 256000).\n";

    // Limpa buffers
    tcflush(serial_port, TCIOFLUSH);

    // Comando GET_INFO: 0xA5 0x50
    unsigned char cmd[2] = { 0xA5, 0x50 };
    int sent = write(serial_port, cmd, 2);
    if (sent != 2) {
        std::cerr << "Erro ao enviar comando\n";
        return 1;
    }

    std::cout << "Comando GET_INFO enviado, aguardando resposta...\n";
    usleep(100000); // 100ms

    unsigned char buffer[64];
    int received = read(serial_port, buffer, sizeof(buffer));
    std::cout << "Recebidos " << received << " bytes:\n";

    for (int i = 0; i < received; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\n");

    close(serial_port);
    return 0;
}
