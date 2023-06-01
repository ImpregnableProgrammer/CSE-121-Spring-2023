#include <stdio.h>
#include "i2c.h"
#include "esp_log.h"
#include "sensor.h"

#define ULTRA_ADDR 0x57

// Ultrasonic sensor guide: https://www.adafruit.com/product/4742

const char *TAG = "lab 7";

void read_ultrasonic_sensor() {
    // Initialize ultranoic sensor ranging session
    // i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    // i2c_master_start(cmd_handle);
    // i2c_master_write_byte(cmd_handle, (ULTRA_ADDR << 1) | I2C_MASTER_READ, true);
    // i2c_master_stop(cmd_handle);
    // i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    // i2c_cmd_link_delete(cmd_handle);

    // Delay
    // vTaskDelay(10 / portTICK_PERIOD_MS);

    // Start ultrasonic ranging session
    // uint8_t bytes[4];
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (ULTRA_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    uint8_t byte;
    cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_read(cmd_handle, &byte, sizeof(byte), I2C_MASTER_ACK);
    i2c_master_stop(cmd_handle);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd_handle, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    ESP_LOGI(TAG, "Byte: %u", byte);
}

void app_main(void)
{ 
    i2c_master_init();
    while (1) {
        read_ultrasonic_sensor();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}   
