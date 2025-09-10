//rpicam-hello
//g++ -o capture capture.cpp -l libcamera -l libcamera-base
//./capture
//g++ -o test_camera test_camera.cpp $(pkg-config --cflags --libs libcamera)
//g++ -o test_camera test_camera.cpp -I/usr/include/libcamera -l libcamera -l libcamera-base
// ./test_camera
#include <libcamera/libcamera.h>
#include <iostream>

using namespace libcamera;

int main() {
    CameraManager cm;
    if (cm.start() != 0) {
        std::cout << "Falha ao iniciar o CameraManager!" << std::endl;
        return -1;
    }

    if (cm.cameras().empty()) {
        std::cout << "Nenhuma câmera detectada!" << std::endl;
        return -1;
    }

    std::cout << "Câmera detectada: " << cm.cameras()[0]->id() << std::endl;

    cm.stop();
    return 0;
}