//sudo apt update
//sudo apt install libcamera-apps



//libcamera-hello



//sudo apt install libopencv-dev
#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Abre a câmera (no Raspberry Pi 4 com libcamera, geralmente é /dev/video0)
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cerr << "Erro: não foi possível abrir a câmera!" << std::endl;
        return -1;
    }

    cv::Mat frame;
    while (true) {
        cap >> frame;  // captura frame
        if (frame.empty()) break;

        cv::imshow("Camera Frontal", frame);

        // Sai se pressionar ESC
        if (cv::waitKey(30) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
// para rodar

// g++ camera.cpp -o camera `pkg-config --cflags --libs opencv4`
//./camera