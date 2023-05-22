# Lab 5 - ESP32 Weather Station using HTTPS
## Introduction
In this lab, we had the ESP32 act as a web client by connecting it over WiFi to an iPhone hotspot, then having it perform an HTTPS request (with TLS handling) to fetch weather data for a certain location from the [wttr.in](wttr.in) website. The location was obtained by performing a GET request to the iPhone web server with a file named `location` in its root directory that contains the name of the location (which must contain _no new lines_). The weather data from wttr.in was then combined with onboard sensor data, which was then POSTed to the iPhone web server.

## References
* Directions for starting a web server on iPhone: https://beebom.com/run-simple-web-server-iphone/
* Example script for creating a simple web server in Python that handles GET and POST requests: https://gist.github.com/mdonkers/63e115cc0c79b4f6b8b3a6b797e485c7