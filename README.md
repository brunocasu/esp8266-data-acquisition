# esp8266-data-aqcuisition
This repository contains projects using Wemos D1 Mini (ESP-8266EX based system) for data collection, using temperature, humdity and CO2 sensors.
![alt text](https://github.com/brunocasu/d1-mini-temp-monitor/blob/master/doc/system_arch.png?raw=true)

## Arduino projects for the ESP8266 using Wemos D1 Mini v4.0.0.0:
### app-sht30-co2-deep-sleep
This project implements a low-power data acquisition system using the ESP8266 Deep Sleep settings (PIN 16 must be connected to RST for this mode to work).

During the initial boot, the system identifies which sensors are present. This first boot is tracked by reading a counter value stored in the RTC memory. For each active sensor detected during the first boot, a corresponding .csv data file is created in the local File System (LittleFS). Subsequent iterations of the program consists in reading data from the configured sensors and storing this data in their respective .csv files.

This application is designed to support up to 2 SHT30 temperature and humidity sensors (connected via I2C), and one SprintIR-WF-100 CO2 sensor (connected via Software Serial).

After completing the data collection process, the device can be configured as a WiFi access point, creating a local network and exposing an FTP server for accessing the File System containing the sensor data files. This mode can be activated by connecting Pin 14 of the D1 Mini board (ESP8266 GPIO 14) to GND. Once the device is in FTP-AP mode, the boot counter value is reset. By deleting the existing files, a new data collection cycle can be initiated with new data files created by the program.

### demo-ftp-ap
Demo project, executes the data acquisition system not in deep sleep mode: allows the RTC count to be tracked, include state variables for the system and sensors, and the execution time of the system in the measurements.

