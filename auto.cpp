#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <windows.h>
#undef min
#include <vector>
#include <numeric>
#include <cmath>
#include "serialib.h"

// --- CONFIGURAÇÕES DA COMUNICAÇÃO ---
#define BAUDRATE 19200
#define SERIAL_PORT "\\\\.\\COM3" // Mude para a sua porta COM correta (ex: "\\\\.\\COM3")

// --- PARÂMETROS DO AUTO-TUNER (para controlo em km/h) ---
#define TUNING_COMMAND 500        // Comando de 0-1000 a ser enviado (500 = ~10.7 km/h)
#define KP_START 10.0f            // Kp inicial (ganhos serão maiores para km/h)
#define KP_STEP 2.0f              // Incremento do Kp
#define SETTLE_TIME_MS 3000       // Tempo para estabilização
#define OSCILLATION_CHECK_MS 5000 // Tempo de observação da oscilação
#define MIN_CYCLES_FOR_TUNE 3     // Requer 3 ciclos completos de oscilação (pico-vale-pico)
#define AMPLITUDE_TOLERANCE 0.25f // Tolerância de 25% na amplitude

serialib serial;

#pragma pack(1)
typedef struct {
    uint8_t cStart = '/';
    int16_t iSpeed;
    int16_t iSteer;
    uint8_t wStateMaster;
    uint8_t wStateSlave;
    float   Kp;
    float   Ki;
    float   Kd;
    uint16_t checksum;
} SerialServer2Hover;
#pragma pack()

#pragma pack(1)
typedef struct {
    uint16_t cStart;
    int16_t iSpeedL; // realSpeed * 100
    int16_t iSpeedR;
    uint16_t iVolt;  // setpoint (speed)
    int16_t iAmpL;   // measured_speed (km/h * 100)
    int16_t iAmpR;
    int32_t iOdomL;
    int32_t iOdomR;
    uint16_t checksum;
} SerialHover2Server;
#pragma pack()

uint16_t CalcCRC(uint8_t *ptr, int count) {
    uint16_t crc = 0;
    while (--count >= 0) {
        crc ^= (uint16_t)(*ptr++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

void HoverSetup(serialib &oSerial, const char *port, int baudrate) {
    char errorOpening = oSerial.openDevice(port, baudrate);
    if (errorOpening != 1) {
        printf("Erro ao abrir a porta serial! Codigo de erro: %d\n", errorOpening);
        exit(errorOpening);
    }
    printf("Conexao bem sucedida com a porta %s\n", port);
}

void HoverSend(serialib &oSerial, int16_t iSpeed, float Kp, float Ki, float Kd) {
    SerialServer2Hover oData;
    oData.iSpeed = iSpeed;
    oData.iSteer = 0;
    oData.wStateMaster = 32;
    oData.wStateSlave = 32;
    oData.Kp = Kp;
    oData.Ki = Ki;
    oData.Kd = Kd;
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2Hover) - 2);
    oSerial.writeBytes((uint8_t*) &oData, sizeof(SerialServer2Hover));
}

bool HoverReceive(serialib &oSerial, SerialHover2Server &oData) {
    static uint8_t buffer[sizeof(SerialHover2Server)];
    static int bytes_received = 0;
    
    while (oSerial.available() > 0) {
        char next_byte_char;
        oSerial.readBytes(&next_byte_char, 1);
        uint8_t next_byte = (uint8_t)next_byte_char;

        if (bytes_received == 0) {
            if (next_byte == 0xCD) {
                buffer[0] = next_byte;
                bytes_received = 1;
            }
        } else if (bytes_received == 1) {
            if (next_byte == 0xAB) {
                buffer[1] = next_byte;
                bytes_received = 2;
            } else {
                bytes_received = 0;
                if (next_byte == 0xCD) {
                    buffer[0] = next_byte;
                    bytes_received = 1;
                }
            }
        } else {
            buffer[bytes_received] = next_byte;
            bytes_received++;
        }
        
        if (bytes_received >= sizeof(SerialHover2Server)) {
            memcpy(&oData, buffer, sizeof(SerialHover2Server));
            bytes_received = 0;

            uint16_t crcReceived = oData.checksum;
            if (crcReceived == CalcCRC((uint8_t*)&oData, sizeof(SerialHover2Server) - 2)) {
                return true;
            } else {
                 printf("\n[Erro de CRC!]");
            }
        }
    }
    return false;
}

int main() {
    HoverSetup(serial, SERIAL_PORT, BAUDRATE);
    SerialHover2Server feedbackData;

    float current_Kp = KP_START;
    float ultimate_Gain_Ku = 0.0f;
    float ultimate_Period_Tu = 0.0f;

    printf("--- INICIANDO AUTO-TUNING PID PARA CONTROLO EM KM/H ---\n");
    printf("AVISO: O motor ira mover-se e oscilar. Garanta que esta numa posicao segura.\n");
    printf("Pressione Enter para comecar...\n");
    getchar();

    while (ultimate_Gain_Ku == 0.0f) {
        printf("\n--------------------------------------------------\n");
        printf("--> Testando com Kp = %.4f (Ki=0, Kd=0)\n", current_Kp);
        
        DWORD settleStartTime = GetTickCount();
        while (GetTickCount() - settleStartTime < SETTLE_TIME_MS) {
            HoverSend(serial, TUNING_COMMAND, current_Kp, 0.0f, 0.0f);
            Sleep(50);
        }

        printf("A observar por oscilacoes durante %dms...\n", OSCILLATION_CHECK_MS);
        
        std::vector<float> peaks, troughs;
        std::vector<DWORD> peakTimes, troughTimes;
        float last_speed = 0;
        bool is_rising = true;
        
        DWORD checkStartTime = GetTickCount();
        while (GetTickCount() - checkStartTime < OSCILLATION_CHECK_MS) {
            HoverSend(serial, TUNING_COMMAND, current_Kp, 0.0f, 0.0f);
            
            if (HoverReceive(serial, feedbackData)) {
                float current_kmh = std::abs((float)feedbackData.iAmpL / 100.0f);
                printf("\rVelocidade Medida: %.2f km/h  ", current_kmh);
                
                if (is_rising && current_kmh < last_speed) {
                    peaks.push_back(last_speed);
                    peakTimes.push_back(GetTickCount());
                    is_rising = false;
                } else if (!is_rising && current_kmh > last_speed) {
                    troughs.push_back(last_speed);
                    troughTimes.push_back(GetTickCount());
                    is_rising = true;
                }
                last_speed = current_kmh;
            }
            Sleep(20);
        }
        printf("\n");

        if (peaks.size() >= MIN_CYCLES_FOR_TUNE && troughs.size() >= MIN_CYCLES_FOR_TUNE) {
            float avg_amplitude = 0;
            for(size_t i = 0; i < peaks.size() && i < troughs.size(); ++i) {
                avg_amplitude += std::abs(peaks[i] - troughs[i]);
            }
            avg_amplitude /= std::min(peaks.size(), troughs.size());
            
            bool stable_oscillation = true;
            for(size_t i = 0; i < peaks.size() && i < troughs.size(); ++i) {
                if (std::abs(std::abs(peaks[i] - troughs[i]) - avg_amplitude) > avg_amplitude * AMPLITUDE_TOLERANCE) {
                    stable_oscillation = false;
                    break;
                }
            }

            if (stable_oscillation) {
                printf("\n>>> OSCILACAO ESTAVEL DETETADA! <<<\n");
                ultimate_Gain_Ku = current_Kp;
                
                float avg_period_ms = 0;
                for (size_t i = 1; i < peakTimes.size(); ++i) {
                    avg_period_ms += (peakTimes[i] - peakTimes[i-1]);
                }
                ultimate_Period_Tu = (avg_period_ms / (peakTimes.size() - 1)) / 1000.0f;
                break;
            } else {
                printf("Oscilacao instavel detetada. A aumentar Kp...\n");
            }
        } else {
            printf("Nao foram detetadas oscilacoes suficientes. A aumentar Kp...\n");
        }
        current_Kp += KP_STEP;
    }

    if (ultimate_Gain_Ku > 0.0f) {
        printf("\n\n--- AUTO-TUNING CONCLUIDO! ---\n\n");
        printf("Valores medidos:\n");
        printf("  - Ganho Final (Ku): %.4f\n", ultimate_Gain_Ku);
        printf("  - Periodo Final (Tu): %.4f segundos\n\n", ultimate_Period_Tu);

        printf("Ganhos PID calculados para controlo em KM/H:\n");
        
        float kp_cl = 0.6f * ultimate_Gain_Ku;
        float ki_cl = (1.2f * ultimate_Gain_Ku) / ultimate_Period_Tu;
        float kd_cl = (0.075f * ultimate_Gain_Ku) * ultimate_Period_Tu;
        printf("  - Regra 'Classic PID':\n    Kp=%.4f, Ki=%.4f, Kd=%.4f\n\n", kp_cl, ki_cl, kd_cl);

        float kp_so = 0.33f * ultimate_Gain_Ku;
        float ki_so = (0.66f * ultimate_Gain_Ku) / ultimate_Period_Tu;
        float kd_so = (0.11f * ultimate_Gain_Ku) * ultimate_Period_Tu;
        printf("  - Regra 'Some Overshoot' (resposta mais rapida):\n    Kp=%.4f, Ki=%.4f, Kd=%.4f\n\n", kp_so, ki_so, kd_so);

        float kp_no = 0.7f * ultimate_Gain_Ku;
        float ki_no = (1.75f * ultimate_Gain_Ku) / ultimate_Period_Tu;
        float kd_no = (0.105f * ultimate_Gain_Ku) * ultimate_Period_Tu;
        printf("  - Regra 'No Overshoot' (mais conservadora):\n    Kp=%.4f, Ki=%.4f, Kd=%.4f\n\n", kp_no, ki_no, kd_no);
        
    } else {
        printf("\n\n--- AUTO-TUNING FALHOU! ---\n");
        printf("Nao foi possivel encontrar uma oscilacao estavel com os parametros atuais.\n");
        printf("Tente aumentar o OSCILLATION_CHECK_MS ou o KP_STEP.\n");
    }

    serial.closeDevice();
    printf("Pressione Enter para sair.\n");
    getchar();
    return 0;
}