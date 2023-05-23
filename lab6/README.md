# Lab 6 - Morse code translator
## Introduction
In this lab, we connected an LED to the Raspberry Pi over GPIO to output messages in Morse code, and the ESP32 was then programmed to translate this LED Morse code output using a photodiode connected to the on-board ADC (Analog-Digital Convertor) channel. The Python script `send` can be used to output morse code over an LED connected to GPIO 5 on the Raspberry Pi by first running `chmod +x send` and then invoking the script as `./send <# repeats> "<Message>"`.

## References
