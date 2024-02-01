# wemos-d1-mini-temp-monitoring
Monitoring system for temperature and relative humidity using Wemos D1 Mini (ESP-8266EX based system) and SHT30 sensor.

## In arduino-prj:
### monitor-ftp-ap
Example project, reading the temperature parameters every cycle using the delay() function and LED indication.
After finishing the measurements, the device can be set as an WiFi access point (local network), exposes an FTP server.
This mode is set by connecting Pin 14 on GND.

### monitor-ftp-ap
Low Power mode using the ESP8266 Light sleep mode.
Data is acquired from the SHT30 every cycle and stored in a local FIle System (LittleFS).
After finishing the measurements, the device can be set as an WiFi access point (local network), and exposes an FTP server where the FS can be accessed.
This mode is set by connecting Pin 14 on GND.
