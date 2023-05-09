#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

// Project for lab 4 - controlling an LCD display LCD1602RGB from DFRobot
// LCD Datasheet: https://www.mouser.com/pdfdocs/DFR0464Datasheet.pdf -- controls LCD display output
// LED driver datasheet: https://www.nxp.com/docs/en/data-sheet/PCA9633.pdf -- controls LCD color and brightness

#define LCD_ADDR 0x7C // Target address for LCD output driver
#define RGB_ADDR 0xC0 // 7-bit target address for LEDs driver

#define I2C_MASTER_SCL_IO 8 // GPIO pin for esp32c3 SCL
#define I2C_MASTER_SDA_IO 10 // GPIO pin for esp32c3 SDA
#define I2C_MASTER_NUM I2C_NUM_0 // Port number (0) for i2c master
#define I2C_MASTER_FREQ_HZ 400000 // i2c SCL operating frequency (400 kHz)

// Initialize i2c interface
void i2c_master_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf); 
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
} 

// Set RGB color of backlight
void setRGB(uint8_t red, uint8_t blue, uint8_t green) {

    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    // i2c_master_start(cmd_handle);
    // i2c_master_write_byte(cmd_handle, RGB_ADDR | I2C_MASTER_WRITE, true);
    // i2c_master_write_byte(cmd_handle, 0xA0 | 0x08, true); // Set auto-increment to on for all registers and start at register to 0x08 (LEDOUT) for turning on LEDs
    // i2c_master_write_byte(cmd_handle, 0xAA, true); // Turn all LED drivers on
    // i2c_master_stop(cmd_handle); // Stop current register write sequence
    // printf("%d\n", i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS));
    // i2c_cmd_link_delete(cmd_handle);

    // cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle); // Start sequence of register writes
    i2c_master_write_byte(cmd_handle, RGB_ADDR | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0xA0 | 0x02, true); // Set auto-increment for only brightness registers (0x2 to 0x5) and start at register 0x2 (LED2)
    i2c_master_write_byte(cmd_handle, blue, true); // Set LED0 (blue) to brightness 255 (max duty cycle)
    i2c_master_write_byte(cmd_handle, green, true); // Set LED1 (green) to brightness 255
    i2c_master_write_byte(cmd_handle, red, true); // Set LED2 (red) to brightness 255
    i2c_master_stop(cmd_handle); // Stop sequence of register writes
    printf("code: %d\n", i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd_handle);
}

// Write string to screen
// 0 < len < 80
void write_string(char* str, uint8_t len) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, LCD_ADDR | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0x80, true); // C0=1, RS=0
    i2c_master_write_byte(cmd_handle, 0x80, true); // Set 6-bit DDRAM address to 0x00
    for (uint8_t i = 0; i < len; ++i) {
        i2c_master_write_byte(cmd_handle, 0xC0, true); // CO=1, RS=1 (write data to RAM)
        i2c_master_write_byte(cmd_handle, str[i], true); // Write character to DDRAM
    }
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
}

// Initialize LCD display
// LCD set to 2-line, 5x8 dot display weith cursor and blinking off
void lcd_init() {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();

    // Initialize LCD
    i2c_master_start(cmd_handle); 
    i2c_master_write_byte(cmd_handle, LCD_ADDR | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, 0x80, true); // CO=1 (not last control byte), RS=0 (for command)
    i2c_master_write_byte(cmd_handle, 0x38, true); // Set display to 2-line/5x8 dot mode
    i2c_master_write_byte(cmd_handle, 0x80, true); // CO=1 (not last control byte), RS=0
    i2c_master_write_byte(cmd_handle, 0x0C, true); // Turn display on, cursor and blinking off
    i2c_master_write_byte(cmd_handle, 0x01, true); // Clear display
    i2c_master_write_byte(cmd_handle, 0x00, true); // CO=0 (last control byte), RS=0
    i2c_master_write_byte(cmd_handle, 0x06, true); // Set cursor shift direction to right, AC increment to +1
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);

    // Set backlight color for LCD to green
    setRGB(0, 255, 0);

    // Output "Hello, world!" by writing character code data to DDRAM
    write_string("Hello, World!", 13);

    // Configure LEDs to show green backlight
    // cmd_handle = i2c_cmd_link_create();
    // i2c_master_start(cmd_handle);
    // i2c_master_write_byte(cmd_handle, RGB_ADDR | I2C_MASTER_WRITE, true);
    // i2c_master_write_byte(cmd_handle, 0xA0 | 0x08, true); // Set auto-increment to on for all registers and start at register to 0x08 (LEDOUT) for turning on LEDs
    // i2c_master_write_byte(cmd_handle, 0xFF, true); // Turn all LED drivers on
    // i2c_master_stop(cmd_handle); // Stop current register write sequence
    // printf("%d\n", i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS));
    // i2c_cmd_link_delete(cmd_handle);
    // cmd_handle = i2c_cmd_link_create();
    // i2c_master_start(cmd_handle); // Start sequence of register writes
    // i2c_master_write_byte(cmd_handle, RGB_ADDR | I2C_MASTER_WRITE, true);
    // i2c_master_write_byte(cmd_handle,0xA0 | 0x02, true); // Set auto-increment for only brightness registers (0x2 to 0x5) and start at register 0x2 (LED2)
    // i2c_master_write_byte(cmd_handle, 255, true); // Set LED0 (blue) to brightness 255 (max duty cycle)
    // i2c_master_write_byte(cmd_handle, 255, true); // Set LED1 (green) to brightness 255
    // i2c_master_write_byte(cmd_handle, 255, true); // Set LED2 (red) to brightness 255
    // i2c_master_stop(cmd_handle); // Stop sequence of register writes
    // printf("code: %d\n", i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS));
    // i2c_cmd_link_delete(cmd_handle);
}

void lcd_task(void *pvParameter) {
    lcd_init();
    while (1) {
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    i2c_master_init();
    // xTaskCreate(lcd_task, "lcd", 4096, NULL, 5, NULL);
    lcd_init();
}
