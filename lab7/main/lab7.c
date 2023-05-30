#include <stdio.h>
#include "i2c.h"
#include "sensor.h"

#define ULTRA_ADDR 0x57

// Ultrasonic sensor guide: https://www.adafruit.com/product/4742

void app_main(void)
{  
    i2c_master_init();
    
}
