#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
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
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
}

// Read IMU gyroscope data into pointer arguments
// Can either read from IMU 1 KB FIFO register OR manually from on-board registers
// The time between samples for the IMU is stored into tDelta
void read_IMU(double *gyro_x, double *gyro_y, double *gyro_z, double *tDelta)
{
    // Write to internal registers
    uint8_t data[1] = {0x0F}; // Choose PWR_MGMT register 0x1F for writing to turn on sensors
    const float factor = 250.0f / (1 << 15); // Gyroscope scaling factor depending on range
    IMU_write_reg(0x1F, data, 1); // Turn on accelerometer and gyroscope sensors
    data[0] = 0x60 | 0x05; // Set gyroscope range = ±250dps, ODR (sampling rate) = 1.6 kHz
    *tDelta = 1 / 1600.0f; // Set tDelta to 1/ODR
    IMU_write_reg(0x20, data, 1);

    // Read gyroscope data and output it
    uint8_t gyro[6];
    IMU_read_reg(0x11, &gyro[0], 1);
    IMU_read_reg(0x12, &gyro[1], 1);
    IMU_read_reg(0x13, &gyro[2], 1);
    IMU_read_reg(0x14, &gyro[3], 1);
    IMU_read_reg(0x15, &gyro[4], 1);
    IMU_read_reg(0x16, &gyro[5], 1);
    *gyro_x = (int16_t)((gyro[0] << 8) | gyro[1]) * factor;
    *gyro_y = (int16_t)((gyro[2] << 8) | gyro[3]) * factor;
    *gyro_z = (int16_t)((gyro[4] << 8) | gyro[5]) * factor;
}

// Create task
void task_sensor(void *pvParameter)
{
    double gyro_x, gyro_y, gyro_z, tDelta;
    double x_angle = 0, y_angle = 0, z_angle = 0;
    const float gyro_thresh = 1; // ±1 threshold for gyroscope measurements
    const float angle_thresh = 1; // ±1 threshold for inclination
    while (1) {
        read_IMU(&gyro_x, &gyro_y, &gyro_z, &tDelta);
        if (gyro_x > gyro_thresh || gyro_x < -gyro_thresh) x_angle += gyro_x * tDelta;
        if (gyro_y > gyro_thresh || gyro_y < -gyro_thresh ) y_angle += gyro_y * tDelta;
        // if (gyro_z > gyro_thresh || gyro_z < -gyro_thresh) z_angle += gyro_z * tDelta; // z angle not needed
        // ESP_LOGI(TAG, "gyro_x: %.2f, gyro_y: %.2f, gyro_z: %.2f", gyro_x, gyro_y, gyro_z); // Output gyroscope values
	      // ESP_LOGI(TAG, "x_angle: %.2f, y_angle: %.2f, z_angle: %.2f", x_angle, y_angle, z_angle); // Output angle values
	      // vTaskDelay(2000 / portTICK_PERIOD_MS); // 2 sec delay
        ESP_LOGI(TAG, "%s%s",
            x_angle > angle_thresh ? "DOWN " : x_angle < -angle_thresh ? "UP " : "",
            y_angle > angle_thresh ? "RIGHT" : y_angle < -angle_thresh ? "LEFT" : ""); // Output inclination direction of board
    }
}

void app_main()
{
    i2c_master_init();
    xTaskCreate(task_sensor, "sensor", 4096, NULL, 5, NULL); // Create task for scheduling in RTOS
}

