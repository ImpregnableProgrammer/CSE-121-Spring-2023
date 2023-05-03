#include "IMU_data.h"

#define IMU_ADDR 0x68 // Slave address for the IMU (ICM-42670-P) on the Rust ESP32-C3 board

#define I2C_MASTER_SCL_IO 8 // GPIO pin for esp32c3 SCL
#define I2C_MASTER_SDA_IO 10 // GPIO pin for esp32c3 SDA
#define I2C_MASTER_NUM I2C_NUM_0 // Port number (0) for i2c master
#define I2C_MASTER_FREQ_HZ 1000000 // i2c master operating frequency (1 MHz)

// IMU datasheet: https://invensense.tdk.com/download-pdf/icm-42670-p-datasheet/

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

// Read data from register on IMU
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
// This method cooses to read from the registers instead of the FIFO since the former is more straightforward
void read_gyro(double *gyro_x, double *gyro_y, double *gyro_z)
{
    // Write to internal registers
    uint8_t data[1]; // Data array
    // data[0] = 0x10;
    // IMU_write_reg(0x02, data, 1); // Software reset for sensor
    data[0] = 0x0C; // Data for turning on gyroscope only in low noise mode
    IMU_write_reg(0x1F, data, 1); // Write data to PR_MGMT register 0x1F
    data[0] = 0x60 | 0x07; // Data to set gyroscope range = ±250dps, ODR (sampling rate) = 400Hz
    IMU_write_reg(0x20, data, 1); // Write to GYRO_CONFIG0 register to configure gyroscope
    const double factor = 250.0f / (1 << 15);

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

void read_accel(double *accel_x, double *accel_y, double *accel_z) {
    uint8_t data[1];
    data[0] = 0x03; // Turn on accelerometer only in low noise mode
    IMU_write_reg(0x1F, data, 1);
    data[0] = 0x20 | 0x07; // Set ±8g range with 400Hz ODR
    IMU_write_reg(0x21, data, 1); // Wrote data to register ACCEL_CONFIG0
    const double factor = 8.0f / (1 << 15); // conversion factor

    // Read and output accelerometer data
    uint8_t accel[6];
    IMU_read_reg(0x0B, &accel[0], 1);
    IMU_read_reg(0x0C, &accel[1], 1);
    IMU_read_reg(0x0D, &accel[2], 1);
    IMU_read_reg(0x0E, &accel[3], 1);
    IMU_read_reg(0x0F, &accel[4], 1);
    IMU_read_reg(0x10, &accel[5], 1);
    *accel_x = (int16_t)((accel[0] << 8) | accel[1]) * factor;
    *accel_y = (int16_t)((accel[2] << 8) | accel[3]) * factor;
    *accel_z = (int16_t)((accel[4] << 8) | accel[5]) * factor;
}


