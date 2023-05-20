#include "i2c.h"

#define SHTC3_ADDR 0x70 // Address for SHTC3 sensor on Rust ESP32-C3 board

void read_temperature_and_humidity(int *temp, int *humidity);

