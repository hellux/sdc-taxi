#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#ifdef PI
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#endif

#define F_SPI 1000000
#define SLAVE "1"

static void spi_tranceive(int fd, void *src, void *dst, int len) {
    struct spi_ioc_transfer transfer = {0};
    transfer.tx_buf = (intptr_t)src;
    transfer.rx_buf = (intptr_t)dst;
    transfer.len = (uint32_t)len;

    ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
}

int main(void) {
    int fd  = open("/dev/spidev0." SLAVE, O_RDWR);
    int freq = F_SPI;
    uint8_t mode = SPI_MODE_0;
    uint8_t bpw = 8;
    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq);

    int packets_sent = 0;
    int packets_lost = 0;

    for (int i = 0; i < 1000; i++) {
        uint8_t cmd = 0x01;
        uint8_t data[] = {0x02, 0x03};
        uint8_t ack = 0x04;
        printf("-----------------------\n");
        printf("%02x | %02x %02x | %02x\n", cmd, data[0], data[1], ack);
        spi_tranceive(fd, (void*)&cmd, (void*)&cmd, 1);
        spi_tranceive(fd, (void*)data, (void*)data, 2);
        spi_tranceive(fd, (void*)&ack, (void*)&ack, 1);

        packets_sent++;
        if (cmd != 0x9a || data[0] != 0x01 || data[1] != 0x02 || ack != 0x3) {
            printf("%02x | %02x %02x | %02x\n", cmd, data[0], data[1], ack);
            packets_lost++;
        }
    }

    printf("packets lost: %d\n", packets_lost);
    printf("packetloss: %.1f%%\n", ((float)packets_lost/(float)packets_sent)*100);

    return 0;
}
