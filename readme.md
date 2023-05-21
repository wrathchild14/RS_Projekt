# Activity detection
## Description of the project
This code implements an activity detection system using an microcontroller and a sensor module. The system reads accelerometer and gyroscope data from the sensor module and classifies the user's activity based on the sensor readings.
## Libraries
```
Wire
Ticker
ESP8266WiFi
ESP8266WebServer.
```
## How to
- Connect the board
- Set up Wi-Fi network credentials (ssid and password) - we used a mobile hotspot
- The board will start sending the detected activity to the specified server address when a change in activity is detected
