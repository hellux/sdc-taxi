int spi_create(const char *dev, int freq);
void spi_destroy(int fd);
void spi_sync(int fd, int len);
void spi_tranceive(int fd, void *src, void *dst, int len);
