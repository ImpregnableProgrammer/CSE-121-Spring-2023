#include "i2c.h"
#include "lcd.h"
#include "sensor.h"

void sensor_task(void *pvParameter) {
    int temp, humidity;
    char temp_string[10], humidity_str[10]; // Last byte is terminating null
    while (true) {
        read_temperature_and_humidity(&temp, &humidity);
        snprintf(temp_string, 10, "Temp: %dC", temp);
        snprintf(humidity_str, 10, "Hum : %d%%", humidity);
        set_line(0);
        write_string(temp_string, 9); // Don't write terminating null byte
        set_line(1);
        write_string(humidity_str, 9);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    i2c_master_init();
    lcd_init();
    xTaskCreate(sensor_task, "sensor write to lcd", 4096, NULL, 1, NULL);
}
