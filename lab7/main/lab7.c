#include <stdio.h>
#include <unistd.h> // for usleep() 
#include <sys/time.h> // For timing
#include <math.h> // For sqrt()
#include "i2c.h"
#include "sensor.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"

// Ultrasonic sensor guide: https://www.adafruit.com/product/4742

const char *TAG = "lab 7";

void config_gpio() {
    // Reset pins
    gpio_reset_pin(GPIO_NUM_0);
    gpio_reset_pin(GPIO_NUM_1);
    gpio_config_t config;

    // Set output pin GPIO 0
    config.pin_bit_mask = 0b01;
    config.intr_type = GPIO_INTR_DISABLE;
    config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&config));

    // Set input pin GPIO 1
    config.pin_bit_mask = 0b10;
    config.mode = GPIO_MODE_INPUT;
    ESP_ERROR_CHECK(gpio_config(&config));
}

void read_ultrasonic_sensor_distance() {
    while (gpio_get_level(GPIO_NUM_1) == 0) {
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_0, 0));
        ESP_ERROR_CHECK(usleep(2));
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_0, 1));
        ESP_ERROR_CHECK(usleep(10));
        ESP_ERROR_CHECK(gpio_set_level(GPIO_NUM_0, 0));
    }
    // Another way for timing, but not needed
    //struct timeval tv1;
    //struct timeval tv2;
    //ESP_ERROR_CHECK(gettimeofday(&tv1, NULL));
    int64_t t1 = esp_timer_get_time();
    while (gpio_get_level(GPIO_NUM_1) == 1);
    // ESP_ERROR_CHECK(gettimeofday(&tv2, NULL));
    int64_t t2 = esp_timer_get_time();
    int temperature;
    read_temperature_and_humidity(&temperature, NULL);
    // Formula: http://hyperphysics.phy-astr.gsu.edu/hbase/Sound/souspe.html
    float speed = (331.4 + 0.6 * temperature) / 10; // in cm/ms
    printf("Distance: %.1f cm at %dC\n", (t2 - t1) * speed / 1000.0f, temperature);
}

void app_main(void)
{   
    i2c_master_init();
    config_gpio();
    while(1) {
        read_ultrasonic_sensor_distance();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}   
