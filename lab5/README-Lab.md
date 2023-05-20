# Lab 5 - Esp32 Weather Station using HTTPS Client with sensor data
## Introduction
In this lab, we had the ESP32 act as a web client by connecting it over WiFi to an iPhone hotspot, then having it perform an HTTPS request to fetch weather data for a certain location from the [wttr.in](wttr.in) website. The location was obtained by performing a GET request to the iPhone acting as a web server with a file named `location` in its root directory that contains the name of the location. The weather data from wttr.in was then combined with onboard sensor data, which was then POSTed to the iPhone web server.

## References
* For starting a web server on iPhone: https://beebom.com/run-simple-web-server-iphone/