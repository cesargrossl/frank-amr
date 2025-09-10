//sudo apt update
//sudo apt install libcamera-apps



//sudo nano /boot/firmware/config.txt
//dtoverlay=bcm2835-v4l2
//sudo reboot
//ls /dev/video*
// v4l2-ctl --list-devices

#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::VideoCapture cap(0);  // /dev/video0
    if (!cap.isOpened()) {
        std::cerr << "Erro ao abrir a câmera!" << std::endl;
        return 1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cv::Mat frame;
    while (true) {
        cap >> frame;
        if (frame.empty()) break;
        cv::imshow("NoIR Camera", frame);
        if (cv::waitKey(1) == 27) break; // ESC para sair
    }
    return 0;
}

// para rodar

// g++ camera.cpp -o camera `pkg-config --cflags --libs opencv4`
//./camera