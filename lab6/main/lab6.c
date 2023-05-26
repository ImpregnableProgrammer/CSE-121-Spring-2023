#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/adc_types.h"
#include "esp_err.h"

// ADC one-shot with auto-calibration documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/adc_oneshot.html

#define MAX_LEN 512
#define SIZE 45
const char *TAG = "lab6";

const char ALPHABET[SIZE] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f','g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z','.',',','?','!','-','/','@','(',')'};

const char *MORSE[SIZE] = {"-----",".----","..---","...--","....-",".....","-....","--...","---..","----.",".-","-...","-.-.","-..",".","..-.","--.","....","..",".---","-.-",".-..","--","-.","---",".--.","--.-",".-.","...","-","..-","...-",".--","-..-","-.--","--..",".-.-.-","--..--","..--..","-.-.--","-....-","-..-.",".--.-.","-.--.","-.--.-"};

void translate(char *buf) {
    //ESP_LOGI(TAG, "buf: %s", buf);
    for (int i = 0; i < SIZE; i++) {
        //ESP_LOGI(TAG, "%c: %s", ALPHABET[i], MORSE[i]);
        if (strcmp(MORSE[i], buf) == 0) {
            putchar(ALPHABET[i]);
            return;
        }
    }
}

void app_main(void)
{
    adc_unit_t UNIT = ADC_UNIT_1;
    adc_channel_t CHANNEL = ADC_CHANNEL_0;
    // ADC resource allocation
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC channel configuration
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CHANNEL, &config));

    // Input calibration
    adc_cali_handle_t cali_handle;
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = UNIT,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle));

    // Read analog input
    int raw = 0, voltage = 0, i = 0, on_count = 0, off_count = 0;
    const int thresh = 1000, delay = 10, dot = 10;
    const int unit = dot / delay;
    char buf[MAX_LEN];
    while (1) {
        // Detection and translation
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, CHANNEL, &raw));
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(cali_handle, raw, &voltage));
        //ESP_LOGI(TAG, "%d mV", voltage);
        if (voltage > thresh) {
            off_count = 0;
            on_count++;
            //printf("on: %d\n", on_count);
        } else if (on_count > 0 || off_count > 0) {
            off_count++;
            //printf("off: %d\n", off_count);
            if (off_count == unit) {
                buf[i++] = on_count > unit ? '-' : '.';
                on_count = 0;
            } else if (off_count == 3 * unit) {
                buf[i++] = '\0';
                translate(buf);
                memset(buf, 0x0, sizeof(buf));
                i = 0;
            } else if (off_count == 6 * unit) {
                putchar(' ');
            } else if (off_count > 12 * unit) {
                off_count = 0;
                putchar('\n');
            }
        }
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}
