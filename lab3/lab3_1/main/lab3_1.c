#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "hal/i2c_types.h"

#define IMU_ADDR 0x68 // Slave address for the IMU (ICM-42670-P) on the Rust ESP32-C3 board

#define I2C_MASTER_SCL_IO 8 // GPIO pin for esp32c3 SCL
#define I2C_MASTER_SDA_IO 10 // GPIO pin for esp32c3 SDA
#define I2C_MASTER_NUM I2C_NUM_0 // Port number (0) for i2c master
#define I2C_MASTER_FREQ_HZ 1000000 // i2c master operating frequency (1 MHz)

// IMU datasheet: https://invensense.tdk.com/download-pdf/icm-42670-p-datasheet/

static const char* TAG = "Lab 3.1";

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

// Write data to register on IMU 
void IMU_write_reg(uint8_t reg, uint8_t* data, size_t size) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (IMU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, reg, true);
    i2c_master_write(cmd_handle, data, size, true);
    i2c_master_stop(cmd_handle);
    printf("write status: %d\n", i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd_handle);
}

void IMU_read_reg(uint8_t reg, uint8_t *buf, size_t size) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (IMU_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, reg, true);
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (IMU_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd_handle, buf, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd_handle);
    printf("read status: %d\n", i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd_handle);
}

// Read IMU accelerometer and gyroscope data into pointer arguments
// Can either read from IMU 1 KB FIFO regsiter OR maunally from on-baprd registers
void read_IMU(int *accel, int *gyro)
{
    // Write to internal registers
    uint8_t data[1] = {0x0F}; // Choose PWR_MGMT register 0x1F for writing to turn on sensors
                              // Then turn on accelerometer and gyroscope sensors
    IMU_write_reg(0x1F, data, 1);

    // Read accelerometer data and output it
    uint8_t ax[2];
    IMU_read_reg(0x0B, &ax[1], 1);
    IMU_read_reg(0x0C, &ax[0], 1);

    printf("accel: %d\n", (ax[1] << 8) + ax[0]);

    (void)accel;
    (void)gyro;
}

// Create task
void task_sensor(void *pvParameter)
{
    int accel = 0, gyro = 0;
    read_IMU(&accel, &gyro); // Throw away initial garbage data
    while (1) {
        read_IMU(&accel, &gyro);
	      ESP_LOGI(TAG, "accel: %d, gyro: %d\n", accel, gyro);
	      vTaskDelay(2000 / portTICK_PERIOD_MS); // 2 sec delay
    }
}

void app_main()
{
    i2c_master_init();
    xTaskCreate(task_sensor, "sensor", 4096, NULL, 5, NULL); // Create task for scheduling in RTOS
}

