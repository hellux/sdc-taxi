#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>

#include "common.h"

#ifdef PI
#include <wiringPi.h>
#include <wiringPiSPI.h>
#endif

#define CHANNEL 0 /* channel to use on RPI */
#define SS 8

#define F_SPI 700000

#define CMD 0xc

static bool transmit(void *msg) {
    bool success = false;
#ifdef PI
    cs_t cs = cs_create(CMD, msg, bc->len);
    digitalWrite(SS, 1);
    digitalWrite(SS, 0);
    wiringPiSPIDataRW(CHANNEL, (unsigned char*)&cs, 1);
    wiringPiSPIDataRW(CHANNEL, (unsigned char*)msg, DATA_LENGTH);
    uint8_t ack;
    wiringPiSPIDataRW(CHANNEL, (unsigned char*)&ack, 1);
    digitalWrite(SS, 1);
    success = (ack == ACK);
#else
    success = ((rand() % 100) < 50);
#endif
    return success;
}

int main(void) {
#ifdef PI
    wiringPiSetupGpio();
    wiringPiSPISetup(CHANNEL, F_SPI);
#endif

    int packets_lost = 0;
    int packets_sent = 0;

    srand(time(NULL));
    uint8_t msg[DATA_LENGTH];

    for (int i = 0; i < 1000; i++) {
        for (int j = 0; j < DATA_LENGTH; j++) {
            msg[j] = rand() % 256;
        }
        if (!transmit((void*)msg)) {
            packets_lost++;
        }
        packets_sent++;
    }

    float loss = (float)packets_lost / (float)packets_sent;
    printf("packet loss: %.1f%%\n", loss*100);

    return 0;
}
