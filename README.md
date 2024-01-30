# wemos-d1-mini-temp-monitoring
Monitoring system for temperature and relative humidity using Wemos D1 Mini (ESP-8266EX based system).

##In arduino-prj:
###monitor-ftp-ap-deep-sleep
This project runs the ESP8266 in Deep sleep mode, reading the temperature parameters every cycle.
After finishing the measurements, the device is set as an WiFi access point (local network), and exposes an FTP server.
