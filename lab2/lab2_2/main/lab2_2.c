#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define SHTC3_ADDR 0x70 // Address for SHTC3 sensor on Rust ESP32-C3 board

#define I2C_MASTER_SCL_IO 8 // GPIO pin for esp32c3 SCL
#define I2C_MASTER_SDA_IO 10 // GPIO pin for esp32c3 SDA
#define I2C_MASTER_NUM I2C_NUM_0 // Port number (0) for i2c master
#define I2C_MASTER_FREQ_HZ 100000 // i2c operating frequency (100 kHz)

// Initialize i2c
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

// Read SHTC3 temperature and humidity data into pointer arguments
void read_temperature_and_humidity(int *temperature, int *humidity)
{
    // Perform wakeup command 0x3517 first
    uint8_t cmd[2] = {0x35, 0x17}; // Wakeup command
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle); 
    i2c_master_write_byte(cmd_handle, (SHTC3_ADDR << 1) | I2C_MASTER_WRITE, true);     
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_master_write(cmd_handle, cmd, sizeof(cmd), true);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
	
    // Send command to read temperature first with clock stretching enabled in normal power mode (command 0x7CA2)
    cmd[0] = 0x7C, cmd[1] = 0xA2; 
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (SHTC3_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd_handle, cmd, sizeof(cmd), true);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);

    // Read temperature and humidity data into pointer arguments
    // temperature data is in (buf[0] << 8) + buf[1]
    // humidity is in (buf[3] << 8) + buf[4]
    // buf[2] and buf[5] are the temperature and humidity checksums
    uint8_t buf[6];
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);	
    i2c_master_write_byte(cmd_handle, (SHTC3_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd_handle, buf, sizeof(buf), I2C_MASTER_ACK);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
	
    // Send sensor sleep command 0xB098
    cmd[0] = 0xB0, cmd[1] = 0x98;
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (SHTC3_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd_handle, cmd, sizeof(cmd), true);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);

    // Perform temperature and humidity reading conversions as specified in datatsheet
    *temperature = ((buf[0] << 8) + buf[1]) * 175 / (1 << 16) - 45;
    *humidity = ((buf[3] << 8) + buf[4]) * 100 / (1 << 16);
}

// Create task
void task_sensor(void *pvParameter)
{
    int temperature = 0, humidity = 0;
    read_temperature_and_humidity(&temperature, &humidity); // Throw away initial garbage data
    while (1) {
        read_temperature_and_humidity(&temperature, &humidity);
	printf("Temperature is %dC (or %dF) with a %d%% humidity\n", temperature, 
			temperature * 9 / 5 + 32, humidity);
	vTaskDelay(2000 / portTICK_PERIOD_MS); // 2 sec delay
    }
}

void app_main()
{
    i2c_master_init();
    xTaskCreate(task_sensor, "sensor", 4096, NULL, 5, NULL); // Create task for scheduling in RTOS
}

