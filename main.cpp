#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <windows.h>

#include "serialib.h"
#include "serialib.cpp"

#define START_FRAME 0xABCD // Start Frame decidido pelo firmware do Hoverboard
#define BAUDRATE 19200 // Baudrate decidido pelo firmware do Hoverboard
#define SEND_MILLIS 50 // Tempo entre os envios de dados para o Hoverboard (em milissegundos)
#define SERIAL_PORT "\\.\\COM3"

serialib serial;

#pragma pack(1) 
typedef struct {
    uint8_t cStart = '/'; // Deve ser 0x2F
    int16_t iSpeed; 
    int16_t iSteer;
    uint8_t wStateMaster;
    uint8_t wStateSlave;
    uint16_t checksum;
} SerialServer2Hover;
#pragma pack()

#pragma pack(1)
typedef struct {
    uint16_t cStart;      // Deve ser 0xABCD
    int16_t iSpeedL;      
    int16_t iSpeedR;     
    uint16_t iVolt;      
    int16_t iAmpL;        
    int16_t iAmpR;        
    int32_t iOdomL;       
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

void HoverSend(serialib &oSerial, int16_t iSteer, int16_t iSpeed, uint8_t wStateMaster = 32, uint8_t wStateSlave = 32) { // Função para enviar os dados para o Hoverboard
    SerialServer2Hover oData;
    oData.iSpeed = (int16_t)iSpeed;
    oData.iSteer = (int16_t)iSteer;
    oData.wStateMaster = wStateMaster;
    oData.wStateSlave = wStateSlave;
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2Hover) - 2);
    oSerial.writeBytes((uint8_t*) &oData, sizeof(SerialServer2Hover));
}

int main() {
    uint8_t wState = 32; // Indica a cor do LED, 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown, 32=Battery3Led, 64=Disable, 128=ShutOff

    char errorOpening = serial.openDevice(SERIAL_PORT, BAUDRATE); // Abre a porta serial
    if (errorOpening != 1) return errorOpening; // Se não conseguir abrir a porta serial, retorna erro
    printf("Conexão bem sucedida com a porta %s\n", SERIAL_PORT); // Imprime mensagem caso a conexão tenha sido bem sucedida

    while (1) {
        unsigned long iNext = 0; // Variável para armazenar o tempo do próximo envio de dados
        unsigned long iNow = GetTickCount(); // Obtém o tempo atual em milissegundos desde que o sistema foi iniciado

        //int16_t iSpeed = 3 * (abs((int)((iNow/20+100) % 400) - 200) - 100);   // varia a velocidade de +300 até -300 e depois para +300 de novo.
        int16_t iSpeed = 150;
        int16_t iSteer = 1 * (abs((int)((iNow/400+100) % 400) - 200) - 100);   // varia a  de +100 até -100 e depois para +100 de novo.
        //int16_t iSteer = -100; 


        if (iNow > iNext) 
        { 
            HoverSend(serial, iSteer, iSpeed, wState, wState); // Envia os dados para o Hoverboard
            iNext = iNow + SEND_MILLIS; // Atualiza o tempo do próximo envio de dados
        }

    }

    serial.closeDevice(); // Fecha a porta serial
    return 0;
}
