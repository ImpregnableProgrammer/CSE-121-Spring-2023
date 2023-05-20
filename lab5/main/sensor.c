#include "sensor.h"

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

