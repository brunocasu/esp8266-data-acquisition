# d1-mini-temp-monitor
This repository contains projects using Wemos D1 Mini (ESP-8266EX based system) for data collection, using temperature, humdity and CO2 sensors.
![alt text](https://github.com/brunocasu/d1-mini-temp-monitor/blob/master/doc/system_arch.png?raw=true)

## Arduino projects for the ESP8266 on Wemos D1 Mini v4.0.0.0:
### app-sht30-co2-deep-sleep
Implements a low Power mode using the ESP8266 Deep sleep settings (PIN 16 Must be connected to RST for this mode to work).
The monitoring system reads the SHT30 temperature and humidity parameters and stored the telemetry data in a local File System (LittleFS).
A .csv data file is created for each of up to 2 SHT30 sensors that can be connected on the I2C buss (device I2C addr. 0x44 and 0x45).
After finishing the measurements, the device can be set as an WiFi access point (local network) exposing an FTP server for accessing the FS.
This mode is set by connecting Pin 14 of the D1 Mini board on GND.


### app-ftp-ap
Demo project, executes the monitoring system not in deep sleep mode (the RTC count can be tracked and the timestamp of the system can be added to the measurements).

