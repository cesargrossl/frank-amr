//rpicam-hello
//g++ -o capture capture.cpp -l libcamera -l libcamera-base
//./capture
#include <libcamera/libcamera.h>
#include <iostream>
#include <thread>

using namespace libcamera;

int main() {
    // Inicializa o CameraManager
    CameraManager cm;
    if (cm.start() != 0) {
        std::cout << "Falha ao iniciar o CameraManager!" << std::endl;
        return -1;
    }

    // Verifica se há câmeras disponíveis
    if (cm.cameras().empty()) {
        std::cout << "Nenhuma câmera detectada!" << std::endl;
        return -1;
    }

    // Obtém a primeira câmera
    std::shared_ptr<Camera> camera = cm.cameras()[0];
    if (camera->acquire() != 0) {
        std::cout << "Falha ao adquirir a câmera!" << std::endl;
        return -1;
    }

    // Configura o stream para visualização
    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({ StreamRole::Viewfinder });
    config->at(0).pixelFormat = PixelFormat::fromString("YUV420");
    config->at(0).size = { 1280, 720 }; // Resolução para visualização
    config->validate();
    camera->configure(config.get());

    // Aloca buffers
    FrameBufferAllocator allocator(camera);
    if (allocator.allocate(config->at(0).stream()) <= 0) {
        std::cout << "Falha ao alocar buffers!" << std::endl;
        return -1;
    }

    // Inicia a câmera
    if (camera->start() != 0) {
        std::cout << "Falha ao iniciar a câmera!" << std::endl;
        return -1;
    }

    // Loop para manter a visualização ativa
    std::cout << "Visualização ativa. Pressione Ctrl+C para encerrar." << std::endl;
    while (true) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request) {
            std::cout << "Falha ao criar solicitação!" << std::endl;
            break;
        }
        request->addBuffer(config->at(0).stream(), allocator.buffers(config->at(0).stream())[0].get());
        camera->queueRequest(request.get());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Para a câmera e libera recursos
    camera->stop();
    camera->release();
    cm.stop();

    return 0;
}