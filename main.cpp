#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <windows.h>

#include "serialib.h"
#include "serialib.cpp"


#define START_FRAME 0xABCD
#define BAUDRATE 19200
#define SEND_MILLIS 100
#define SERIAL_PORT "\\.\\COM3"

serialib serial;

typedef struct {
    uint16_t cStart;
    int16_t iSpeed;
    int16_t iSteer;
    uint8_t wStateMaster;
    uint8_t wStateSlave;
    uint16_t checksum;
} SerialServer2Hover;

uint16_t cStart = '/';
int16_t  iSpeed = 0;
int16_t  iSteer = 0;
uint8_t  wStateMaster = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff
uint8_t  wStateSlave = 0;   // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown   , 32=Battery3Led, 64=Disable, 128=ShutOff

typedef struct {
    uint16_t cStart;
    int16_t iSpeedL;
    int16_t iSpeedR;
    uint16_t iVolt;
    int16_t iAmpL;
    int16_t iAmpR;
    int32_t iOdomL;
    int32_t iOdomR;
    uint16_t checksum;
} SerialHover2Server;

uint16_t CalcCRC(uint8_t *ptr, int count) {
    uint16_t crc = 0;
    while (--count >= 0) {
        crc ^= (uint16_t)(*ptr++) << 8;
        for (uint8_t i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }
    return crc;
}

template <typename O, typename I> void HoverSend(O& oSerial, I iSteer, I iSpeed, uint8_t wStateMaster = 32, uint8_t wStateSlave = 0) {
    SerialServer2Hover oData;
    oData.iSpeed    = (int16_t)iSpeed;
    oData.iSteer    = (int16_t)iSteer;
    oData.wStateMaster  = wStateMaster;
    oData.wStateSlave   = wStateSlave;
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2Hover)-2); // first bytes except crc

    oSerial.writeBytes((uint8_t*)&oData, sizeof(SerialServer2Hover));
    printf("Speed=%d, Steer=%d, StateMaster=%d, StateSlave=%d\n", iSpeed, iSteer, wStateMaster, wStateSlave);
}

/*
template <typename O, typename I>
void HoverSend(O& oSerial, I iSteer, I iSpeed, uint8_t wStateMaster = 32, uint8_t wStateSlave = 0) {
    SerialServer2Hover oData = {0xABCD, (int16_t)iSpeed, (int16_t)iSteer, wStateMaster, wStateSlave, 0};
    oData.checksum = CalcCRC((uint8_t*)&oData, sizeof(SerialServer2Hover) - 2);
    
    oSerial.writeBytes((uint8_t*)&oData, sizeof(SerialServer2Hover));
    printf("Sent: Speed=%d, Steer=%d, StateMaster=%d, StateSlave=%d\n", iSpeed, iSteer, wStateMaster, wStateSlave);
}
*/

int main() {
    char errorOpening = serial.openDevice(SERIAL_PORT, BAUDRATE);
    if (errorOpening!=1) return errorOpening;
    printf ("Successful connection to %s\n",SERIAL_PORT);

    uint8_t wState = 1;
    unsigned long iNext = 0;

    while (1) {
        unsigned long iNow = GetTickCount();
        //int16_t iSpeed = 3 * (abs((int)((iNow / 20 + 100) % 400) - 200) - 100);
        int16_t iSpeed = 170;
        int16_t iSteer = 1 * (abs((int)((iNow / 400 + 100) % 400) - 200) - 100);
        
        if (iNow > iNext) {
            HoverSend(SERIAL_PORT, iSteer, iSpeed, wState, wState);
            iNext = iNow + SEND_MILLIS;
        }
        Sleep(1000);
    }
    serial.closeDevice();
    return 0;
}