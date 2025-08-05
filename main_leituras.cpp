#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <windows.h> // Para GetTickCount()

#include "serialib.h"

#define START_FRAME 0xABCD // Start Frame decidido pelo firmware do Hoverboard
#define BAUDRATE 19200 // Baudrate decidido pelo firmware do Hoverboard
#define SEND_MILLIS 5 // Tempo entre os envios de dados para o Hoverboard (em milissegundos)
#define SERIAL_PORT "\\.\\COM3"
#define MAX_MEASUREMENTS 100 // Número máximo de medidas a serem lidas

serialib serial;

// Variável global para armazenar o tempo de início da primeira medida
// Usamos static para garantir que ela seja inicializada uma vez e mantenha seu valor
static unsigned long initialMeasurementTickCount = 0;

#pragma pack(1)
typedef struct {
    uint8_t cStart = '/'; // Deve ser 0x2F
    int16_t iSpeed;      // 100* km/h
    int16_t iSteer;
    uint8_t wStateMaster; // Indica a cor do LED na master, 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown, 32=Battery3Led, 64=Disable, 128=ShutOff
    uint8_t wStateSlave;  // Indica a cor do LED na slave
    float   Kp;
    float   Ki;
    float   Kd;
    uint16_t checksum;
} SerialServer2Hover;
#pragma pack()

#pragma pack(1)
typedef struct {
    uint16_t cStart;       // Deve ser 0xABCD
    int16_t iSpeedL;
    int16_t iSpeedR;
    uint16_t iVolt;        // 100* V
    int16_t iAmpL;
    int16_t iAmpR;         // 100* A
    int32_t iOdomL;        // hall steps
    int32_t iOdomR;
    uint16_t checksum;
} SerialHover2Server;
#pragma pack()

uint16_t CalcCRC(uint8_t *ptr, int count) { // Função para cálculo do checksum, utilizado nos pacotes enviados e recebidos para o Hoverboard
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
    char errorOpening = oSerial.openDevice(port, baudrate); // Abre a porta serial
    if (errorOpening != 1) {
        printf("Erro ao abrir a porta serial! Código de erro: %d\n", errorOpening); // Se não conseguir abrir a porta serial, retorna erro
        exit(errorOpening);
    }
    printf("Conexão bem sucedida com a porta %s\n", port); // Imprime mensagem caso a conexão tenha sido bem sucedida
}

void HoverSend(serialib &oSerial, int16_t iSteer, int16_t iSpeed, uint8_t wStateMaster, uint8_t wStateSlave, float Kp, float Ki, float Kd) { // Função para enviar os dados para o Hoverboard
    SerialServer2Hover oData;
    oData.iSpeed = (int16_t)iSpeed;
    oData.iSteer = (int16_t)iSteer;
    oData.wStateMaster = wStateMaster;
    oData.wStateSlave = wStateSlave;
    oData.Kp = Kp;
    oData.Ki = Ki;
    oData.Kd = Kd;
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2Hover) - 2);
    oSerial.writeBytes((uint8_t*) &oData, sizeof(SerialServer2Hover));
}

void HoverReceive(serialib &oSerial, SerialHover2Server &oData, int *measurementCount) {
    uint8_t buffer[sizeof(SerialHover2Server)];
    int bytesRead = oSerial.readBytes(buffer, sizeof(SerialHover2Server));

    if (bytesRead == sizeof(SerialHover2Server)) {
        memcpy(&oData, buffer, sizeof(SerialHover2Server));

        uint16_t crcReceived = oData.checksum;
        oData.checksum = 0;

        uint16_t crcCalculated = CalcCRC((uint8_t*)&oData, sizeof(SerialHover2Server) - 2);

        if (crcReceived == crcCalculated) {
            unsigned long currentTimeMillis = GetTickCount();

            // Se for a primeira medida, armazena o tempo inicial
            if (*measurementCount == 0) {
                initialMeasurementTickCount = currentTimeMillis;
            }

            // Calcula o tempo decorrido desde a primeira medida
            unsigned long elapsedTimeMillis = currentTimeMillis - initialMeasurementTickCount;


            unsigned long seconds = (elapsedTimeMillis / 1000) % 60;
            unsigned long milliseconds = elapsedTimeMillis % 1000;

            printf("Speed: %d, Measured_speed: %d, Time: %02lu.%03lu\n",
                   oData.iVolt, -(oData.iSpeedL), seconds, milliseconds);
                   //oData.iVolt, -(oData.iAmpL), seconds, milliseconds);
            (*measurementCount)++;
        } else {
            printf("Erro de CRC, dados corrompidos!\n");
        }
    }
}

int main() {
    uint8_t wState = 32;
    int measurementCount = 0;

    HoverSetup(serial, SERIAL_PORT, BAUDRATE);

    SerialHover2Server oData;
    unsigned long iNext = 0;

    while (measurementCount < MAX_MEASUREMENTS) {
        unsigned long iNow = GetTickCount();

        int16_t iSpeed = 300;
        int16_t iSteer = 0;
        float Kp_s = 30.0f; // 60.0f para filtro
        float Ki_s = 85.0f; //85
        float Kd_s = 5.0f;

        if (iNow > iNext) {
            HoverSend(serial, iSteer, iSpeed, wState, wState, Kp_s, Ki_s, Kd_s);
            iNext = iNow + SEND_MILLIS;
        }

        HoverReceive(serial, oData, &measurementCount);
    }

    printf("100 medidas coletadas. Encerrando...\n");
    serial.closeDevice();
    return 0;
}