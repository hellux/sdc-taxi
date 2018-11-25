#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>

#include "common.h"

#ifdef PI
#include <wiringPi.h>
#include <wiringPiSPI.h>
#endif

#define F_SPI 1000000

int main(void) {
    wiringPiSetup();
    wiringPiSPISetup(0, F_SPI);

    int packets_lost = 0;
    int packets_sent = 0;

    for (int i = 0; i < 10; i++) {
        uint8_t cmd = 0x01;
        uint8_t data[] = {0x02, 0x03};
        uint8_t ack = 0x04;
        printf("-----------------------\n");
        printf("%02x | %02x %02x | %02x\n", cmd, data[0], data[1], ack);
        wiringPiSPIDataRW(0, (unsigned char*)&cmd, 1);
        wiringPiSPIDataRW(0, (unsigned char*)data, 2);
        wiringPiSPIDataRW(0, (unsigned char*)&ack, 1);
        printf("%02x | %02x %02x | %02x\n", cmd, data[0], data[1], ack);
    }

    packets_lost++;
    packets_sent++;

    float loss = (float)packets_lost / (float)packets_sent;
    printf("packet loss: %.2f%%\n", loss*100);

    return 0;
}
