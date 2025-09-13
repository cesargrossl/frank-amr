#include <iostream>
#include <pigpio.h>
#include <unistd.h>

#define TRIG 23  // GPIO23 (pino físico 16)
#define ECHO 24  // GPIO24 (pino físico 18)

// Constantes do HC-SR04
#define MIN_DISTANCE 2.0    // Distância mínima confiável (cm)
#define MAX_DISTANCE 400.0  // Distância máxima teórica (cm)
#define TIMEOUT_US 30000    // Timeout em microssegundos (~5m)

using namespace std;

enum SensorStatus {
    LEITURA_OK,
    MUITO_PERTO,
    MUITO_LONGE,
    TIMEOUT_ECHO_LOW,
    TIMEOUT_ECHO_HIGH,
    ERRO_GERAL
};

struct MedicaoResult {
    float distancia;
    SensorStatus status;
    string mensagem;
};

MedicaoResult medir_distancia() {
    MedicaoResult result = {0.0, ERRO_GERAL, ""};
    
    // Pulso no TRIG
    gpioWrite(TRIG, 0);
    gpioDelay(2);
    gpioWrite(TRIG, 1);
    gpioDelay(10);
    gpioWrite(TRIG, 0);

    // Espera início do pulso no ECHO
    uint32_t inicio_tempo = gpioTick();
    uint32_t timeout_start = inicio_tempo;
    
    while (gpioRead(ECHO) == 0) {
        inicio_tempo = gpioTick();
        // Timeout de ~38ms para objetos muito distantes
        if ((inicio_tempo - timeout_start) > 38000) {
            result.status = TIMEOUT_ECHO_LOW;
            result.mensagem = "Timeout: ECHO não subiu (objeto muito distante ou sem reflexão)";
            return result;
        }
    }

    // Mede duração do pulso alto
    uint32_t fim_tempo = inicio_tempo;
    uint32_t timeout_high = inicio_tempo;
    
    while (gpioRead(ECHO) == 1) {
        fim_tempo = gpioTick();
        // Timeout para evitar travamento
        if ((fim_tempo - timeout_high) > TIMEOUT_US) {
            result.status = TIMEOUT_ECHO_HIGH;
            result.mensagem = "Timeout: ECHO não desceu (possível interferência)";
            return result;
        }
    }

    // Calcula distância
    double duracao = fim_tempo - inicio_tempo; // microssegundos
    float distancia = (duracao * 0.0343) / 2;  // cm
    
    result.distancia = distancia;
    
    // Análise da leitura
    if (distancia < MIN_DISTANCE) {
        result.status = MUITO_PERTO;
        result.mensagem = "Objeto muito próximo (< " + to_string((int)MIN_DISTANCE) + " cm) - leitura pode ser imprecisa";
    }
    else if (distancia > MAX_DISTANCE) {
        result.status = MUITO_LONGE;
        result.mensagem = "Objeto muito distante (> " + to_string((int)MAX_DISTANCE) + " cm) - fora do alcance confiável";
    }
    else {
        result.status = LEITURA_OK;
        result.mensagem = "Leitura normal";
    }
    
    return result;
}

void imprimir_resultado(const MedicaoResult& result) {
    switch(result.status) {
        case LEITURA_OK:
            cout << "Distância: " << fixed << setprecision(1) << result.distancia << " cm" << endl;
            break;
            
        case MUITO_PERTO:
            cout << "⚠️  MUITO PERTO: " << fixed << setprecision(1) << result.distancia 
                 << " cm - " << result.mensagem << endl;
            break;
            
        case MUITO_LONGE:
            cout << "📏 MUITO LONGE: " << fixed << setprecision(1) << result.distancia 
                 << " cm - " << result.mensagem << endl;
            break;
            
        case TIMEOUT_ECHO_LOW:
            cout << "🚫 ERRO: " << result.mensagem << endl;
            cout << "   Possíveis causas: objeto > 4m, sem superfície refletiva, ou conexões soltas" << endl;
            break;
            
        case TIMEOUT_ECHO_HIGH:
            cout << "⚡ ERRO: " << result.mensagem << endl;
            cout << "   Possíveis causas: interferência elétrica ou problema no sensor" << endl;
            break;
            
        case ERRO_GERAL:
            cout << "❌ ERRO GERAL na medição" << endl;
            break;
    }
}

int main() {
    if (gpioInitialise() < 0) {
        cerr << "Erro ao iniciar pigpio" << endl;
        return 1;
    }

    gpioSetMode(TRIG, PI_OUTPUT);
    gpioSetMode(ECHO, PI_INPUT);
    
    cout << "=== Sensor HC-SR04 - Detecção Inteligente ===" << endl;
    cout << "Alcance confiável: " << MIN_DISTANCE << " - " << MAX_DISTANCE << " cm" << endl;
    cout << "Pressione Ctrl+C para sair\n" << endl;

    int contador_erros = 0;
    
    while (true) {
        MedicaoResult result = medir_distancia();
        
        cout << "[" << contador_erros << " erros] ";
        imprimir_resultado(result);
        
        // Contador de erros consecutivos
        if (result.status != LEITURA_OK) {
            contador_erros++;
            if (contador_erros > 5) {
                cout << "\n⚠️  ATENÇÃO: Muitos erros consecutivos!" << endl;
                cout << "   Verifique as conexões e alimentação do sensor" << endl;
                contador_erros = 0; // Reset
            }
        } else {
            contador_erros = 0; // Reset em leitura bem-sucedida
        }
        
        usleep(500000); // 500ms
    }

    gpioTerminate();
    return 0;
}