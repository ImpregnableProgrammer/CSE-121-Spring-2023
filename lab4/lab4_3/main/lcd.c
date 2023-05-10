#include "lcd.h"

// LCD Datasheet: https://www.mouser.com/pdfdocs/DFR0464Datasheet.pdf -- controls LCD display output
// LED driver datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9633.pdf -- controls LCD color and brightness

// Initialize LCD display
// LCD set to 2-line, 5x8 dot display with cursor and blinking off
void lcd_init() {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle); 
    i2c_master_write_byte(cmd_handle, LCD_ADDR | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0x80, true); // CO=1 (not last control byte), RS=0 (for command)
    i2c_master_write_byte(cmd_handle, 0x38, true); // Set display to 2-line/5x8 dot mode
    i2c_master_write_byte(cmd_handle, 0x80, true); // CO=1 (not last control byte), RS=0
    i2c_master_write_byte(cmd_handle, 0x0C, true); // Turn display on, cursor and blinking off
    i2c_master_write_byte(cmd_handle, 0x80, true); // CO=1 (not last control byte), RS=0
    i2c_master_write_byte(cmd_handle, 0x01, true); // Clear display and perform display and DDRAM reset
    i2c_master_write_byte(cmd_handle, 0x00, true); // CO=0 (last control byte), RS=0
    i2c_master_write_byte(cmd_handle, 0x06, true); // Set cursor shift direction to right, Addr increment to +1
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
}

// Set RGB color of backlight
// 0 <= red, green, blue <= 255
void setRGB(uint8_t red, uint8_t green, uint8_t blue) {

    i2c_cmd_handle_t cmd_handle;
    
    // Get out of sleep mode
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, RGB_ADDR | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0x80 | 0x00, true); // Access register 0x00 (MODE1) for main settings
    i2c_master_write_byte(cmd_handle, 0x01, true); // Get LEDs out of SLEEP mode
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);

    vTaskDelay(10 / portTICK_PERIOD_MS); // Wait for oscillator to activate

    // Turn LED drivers on first
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, RGB_ADDR | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0xA0 | 0x08, true); // Set auto-increment on for all register addresses and go to register to 0x08 (LEDOUT) for turning on LEDs
    i2c_master_write_byte(cmd_handle, 0xFF, true); // Turn all LED drivers on
    i2c_master_stop(cmd_handle); // Stop current register write sequence
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    
    // Set LED values
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle); // Start sequence of register writes
    i2c_master_write_byte(cmd_handle, RGB_ADDR | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0xA0 | 0x02, true); // Set auto-increment for only brightness registers (0x2 to 0x5) and start at register 0x2 (LED2)
    i2c_master_write_byte(cmd_handle, blue, true); // Set LED0 (blue) to brightness given
    i2c_master_write_byte(cmd_handle, green, true); // Set LED1 (green) to brightness given
    i2c_master_write_byte(cmd_handle, red, true); // Set LED2 (red) to brightness given
    i2c_master_stop(cmd_handle); // Stop sequence of register writes
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
}

// Set cursor to line given % 2
void set_line(uint8_t line) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, LCD_ADDR | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0x80, true); // CO=1, RS=0 (write DDRAM address)
    i2c_master_write_byte(cmd_handle, 0x80 | (0x40 * (line % 2)), true); // Set 6-bit DDRAM address
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
}

// Write string to screen
// 0 < len <= 16
void write_string(char* str, uint8_t len) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, LCD_ADDR | I2C_MASTER_WRITE, true);
    for (uint8_t i = 0; i < len; ++i) {
        i2c_master_write_byte(cmd_handle, 0xC0, true); // CO=1, RS=1 (write data to DDRAM)
        i2c_master_write_byte(cmd_handle, str[i], true); // Write character to DDRAM
    }
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
}
