#include "spi.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

int spi_create(const char *dev, int freq) {
    int fd = open(dev, O_RDWR);
    int mode = SPI_MODE_0;
    int bpw = 8;

    ioctl(fd, SPI_IOC_WR_MODE, &mode);
    ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw);
    ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq);

    return fd;
}

void spi_destroy(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}

void spi_sync(int fd, int length) {
    int cmd = 0;
    for (int i = 0; i < length; i++) {
        spi_tranceive(fd, (void*)&cmd, NULL, 1);
    }
}

void spi_tranceive(int fd, void *src, void *dst, int len) {
    struct spi_ioc_transfer transfer = {0};
    transfer.tx_buf = (intptr_t)src;
    transfer.rx_buf = (intptr_t)dst;
    transfer.len = (long)len;

    ioctl(fd, SPI_IOC_MESSAGE(1), &transfer);
}
