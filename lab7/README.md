# Lab 7 - Ultrasonic sensor with Temperature Adjusted readings
## Introduction
In this lab, we used the RCWL-1601 HC-SR04 compatible ultrasonic sensor to measure the distance to an object in centimeters with the readings adjusted for the current temperature of the environment given by the temperature sensor on the ESP32 board. The ultrasonic distance sensor sends out ultrasonic pulses triggered by the TRIG pin, and the ECHO pin then remains high until an echo is received. The duration of the ECHO pin remaining high can then be used to calculate the distance to an object.

## References
* RCWL-1601 Ultrasonic sensor page: https://www.adafruit.com/product/4007
* HC-SR04 Ultrasonic sensor guide: https://lastminuteengineers.com/arduino-sr04-ultrasonic-sensor-tutorial/
* ESP32-C3 timer documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/system/esp_timer.html?highlight=timer#_CPPv418esp_timer_get_timev

