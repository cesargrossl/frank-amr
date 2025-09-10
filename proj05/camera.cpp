sudo apt update
sudo apt install -y libcamera-apps libcamera-dev g++ cmake pkg-config \
                    libopencv-dev gstreamer1.0-tools gstreamer1.0-libcamera \
                    gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
                    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly



libcamera-hello --list-cameras
# ou
rpicam-hello --list-cameras



libcamera-hello -t 0      # ou rpicam-hello -t 0 (preview contínuo)



#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Pipeline GStreamer: libcamera -> (converte) -> appsink (OpenCV)
    // Ajuste width/height/framerate conforme sua necessidade
    const std::string pipeline =
        "libcamerasrc ! "
        "video/x-raw, width=1280, height=720, framerate=30/1 ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true sync=false";

    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::cerr << "Erro: não abriu a câmera via GStreamer/libcamera.\n"
                     "Verifique se o pacote gstreamer1.0-libcamera está instalado\n"
                     "e se seu OpenCV foi compilado com suporte a GStreamer.\n";
        return 1;
    }

    cv::Mat frame;
    while (true) {
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Falha ao capturar frame.\n";
            break;
        }
        cv::imshow("Pi Camera v2 (IMX219) - Preview", frame);
        // ESC ou 'q' para sair
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q') break;
    }
    return 0;
}

sudo apt update
sudo apt install -y libopencv-dev


g++ -std=c++17 cam.cpp -o cam \  $(pkg-config --cflags --libs opencv4 gstreamer-1.0 gstreamer-app-1.0)


Package 'gstreamer-1.0' was not found in the pkg-config search path.
Package 'gstreamer-app-1.0' was not found


sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

