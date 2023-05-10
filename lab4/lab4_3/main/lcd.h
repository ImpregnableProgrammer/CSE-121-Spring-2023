#include "i2c.h"

#define LCD_ADDR 0x7C // Target address for LCD output driver
#define RGB_ADDR 0xC0 // 7-bit target address for LEDs driver

void lcd_init();
void setRGB(uint8_t red, uint8_t green, uint8_t blue);
void set_line(uint8_t line);
void write_string(char *str, uint8_t len);
