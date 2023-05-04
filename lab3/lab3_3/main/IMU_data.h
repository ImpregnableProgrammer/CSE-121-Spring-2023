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

// i2c intialization method
void i2c_master_init();

// Turn on IMU sensors
void startIMU();

// IMU gyroscope data fetch method
// Read and return gyroscope data with ±250dps range and 50Hz (=25*2=50Hz) ODR (sampling rate)
// with 25 Hz low pass filter bandwidth
// See Shannon sampling theorem: https://en.wikipedia.org/wiki/Nyquist%E2%80%93Shannon_sampling_theorem
void read_gyro(double *gyro_x, double *gyro_y, double *gyro_z);

// IMU accelerometer data fetch method
// Read and return accelerometer data with ±4g range and 400Hz ODR
// with 180 Hz low pass filter bandwidth
void read_accel(double *accel_x, double *accel_y, double *accel_z);
