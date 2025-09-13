#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdint>

using namespace std;

int main() {
    int serial_port = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_port < 0) {
        cerr << "Erro ao abrir /dev/serial0" << endl;
        return 1;
    }

    termios tty{};
    tcgetattr(serial_port, &tty);

    cfsetispeed(&tty, B9600); // Baudrate do JSN-SR04M (normalmente 9600)
    cfsetospeed(&tty, B9600);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tcsetattr(serial_port, TCSANOW, &tty);

    while (true) {
        uint8_t buffer[4];
        int n = read(serial_port, buffer, sizeof(buffer));
        if (n == 4 && buffer[0] == 0xFF) { // protocolo do JSN-SR04M UART
            int distancia = (buffer[1] << 8) | buffer[2];
            cout << "Distancia: " << distancia << " mm" << endl;
        }
    }

    close(serial_port);
    return 0;
}



sudo apt-get install pigpio
g++ dist.cpp -o dist -lpigpio -lpthread
sudo pigpiod
./dist
