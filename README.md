# d1-mini-temp-monitor
Monitoring system for temperature and relative humidity using Wemos D1 Mini (ESP-8266EX based system) and SHT30 sensor.

## Arduino projects for the ESP8266 on Wemos D1 Mini v4.0.0.0:
### monitor-ftp-ap
Example project, runs a loop where every iteration the SHT30 temperature and humidity parameters are read and stored in a local File System (LittleFS).
A .csv data file is created for each of 2 SHT30 sensors that can be connected on the I2C buss (addr. 0x44 and 0x45).
After finishing the measurements, the device can be set as an WiFi access point (local network) exposing an FTP server for accessing the FS.
This mode is set by connecting Pin 14 of the D1 Mini board on GND.

### monitor-ftp-ap-deep-sleep
Project suited for production environment. Implements a low Power mode using the ESP8266 Deep sleep settings (PIN 16 Must be connected to RST for this mode to work).
