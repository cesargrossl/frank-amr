//rpicam-hello
//g++ -o capture capture.cpp -l libcamera -l libcamera-base
//./capture
#include <libcamera/libcamera.h>
#include <iostream>
#include <fstream>

using namespace libcamera;

int main() {
    // Inicializa a CameraManager
    CameraManager cm;
    cm.start();

    // Verifica se há câmeras disponíveis
    if (cm.cameras().empty()) {
        std::cout << "Nenhuma câmera detectada!" << std::endl;
        return -1;
    }

    // Obtém a primeira câmera disponível
    std::shared_ptr<Camera> camera = cm.cameras()[0];
    if (camera->acquire() != 0) {
        std::cout << "Falha ao adquirir a câmera!" << std::endl;
        return -1;
    }

    // Configura o stream da câmera
    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({ StreamRole::StillCapture });
    config->at(0).pixelFormat = PixelFormat::fromString("YUV420");
    config->at(0).size = { 1920, 1080 }; // Resolução desejada
    config->validate();
    camera->configure(config.get());

    // Aloca buffers para o stream
    FrameBufferAllocator allocator(camera);
    allocator.allocate(config->at(0).stream());

    // Cria uma solicitação de captura
    std::unique_ptr<Request> request = camera->createRequest();
    const auto& stream = config->at(0).stream();
    request->addBuffer(stream, allocator.buffers(stream)[0].get());

    // Inicia a câmera
    camera->start();
    camera->requestCompleted.connect([](Request* req) {
        if (req->status() == Request::RequestComplete) {
            std::cout << "Captura concluída!" << std::endl;
        }
    });

    // Enfileira a solicitação de captura
    camera->queueRequest(request.get());

    // Aguarda a captura (simplificado)
    sleep(2);

    // Para a câmera e libera recursos
    camera->stop();
    camera->release();
    cm.stop();

    // Salva a imagem (exemplo básico, precisa de mais código para processar os dados do buffer)
    std::cout << "Imagem capturada. Processamento do buffer necessário para salvar." << std::endl;

    return 0;
}