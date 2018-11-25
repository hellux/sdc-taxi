#include <stdint.h>
#include <stdbool.h>

typedef uint8_t cs_t;

uint8_t cs_cmd(cs_t cs);
cs_t cs_create(uint8_t cmd, void *data, int len);
bool cs_check(cs_t cs, void *data, int len);

#define ACK 0xc7
#define DATA_LENGTH 10
