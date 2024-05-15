/*
 * app-ftp-ap-sht30.ino
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of
 * the License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If not,
 * see <https://www.gnu.org/licenses/>.
 *
 *  Created on: Jan 25, 2024
 *      Author: Bruno Casu
 *
 *  Version 1.1 (Apr 10, 2024)
 */
 
#include <ESP8266WiFi.h>
#include <PolledTimeout.h>
#include <LittleFS.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SimpleFTPServer.h> // Replacing <ESP8266FtpServer.h>
#include <cozir.h>
#include <sht30.h>

#define SERIAL_SPEED  115200
#define APP_VERSION "app-ftp-ap-sht30 V1.1"

// Defines for the data aquisition system
#define SHT30_I2C_ADDR_PIN_HIGH 0x45  // Jumper NOT connected
#define SHT30_I2C_ADDR_PIN_LOW 0x44  // Jumper connected
#define SLEEP_TIME_MS 1800000  // Interval between measurements in ms
#define GPIO_SET_ACCESS_POINT 14 // On Wemos D1 Mini - Pin number 14 (GPIO14)

// Remove when deploying in production environment
//#define DEBUG_MODE

#define CYCLE_COUNTER_RST 10000000  // When resetig via GPIO, RTC memory gets wrong counter value - must be reset
unsigned long cycle_counter = 0;

// Temperature and humidity sensor
SHT30 sht30_1_handler(SHT30_I2C_ADDR_PIN_HIGH); // Addr 0x45
SHT30 sht30_2_handler(SHT30_I2C_ADDR_PIN_LOW); // Addr 0x44

// WiFi Access Point configuration
IPAddress local_IP(10,10,10,1); // FTP server address
IPAddress gateway(10,10,10,1);
IPAddress subnet_mask(255,255,255,0);
const char* ssid_AP = "AP-ESP8266"; // No password set

// FTP server access configuration
const char* user_FTP = "esp8266";
const char* pwd_FTP = "esp8266";
FtpServer ftpSrv; // Handler

// LittleFs info
FSInfo fs_info;

// Data file configuration
const char* sht30_1_file_path = "/sht30_addr_45_data.csv";
const char* sht30_2_file_path = "/sht30_addr_44_data.csv";
const char* sht30_csv_header_description = "Ctr;Temperature(C);RelHumidity(RH%)"; // Added at the creation of the Data file


/** PF definitions **/

/**
 * Configures the ESP8266 as WiFi Access Point, and starts the FTP server
 */
void setAccessPoint() {
  Serial.begin(SERIAL_SPEED);
  delay(100);
  // Configure WiFi Access point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid_AP); // No password for connecting
  WiFi.softAPConfig(local_IP, gateway, subnet_mask);
  // Device Info
  Serial.print("\n-->SW VERSION: ");
  Serial.println(APP_VERSION);
  Serial.print("\n-->SLEEP_TIME_MS: ");
  Serial.println(SLEEP_TIME_MS);
  Serial.print("\n-->AP Mode - WiFi SSID (NO PASSWORD): ");
  Serial.println(ssid_AP);
  Serial.print("\n-->FTP Server IP Addr: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Plain FTP Connection (insecure)\nFTP_USR: ");
  Serial.println(user_FTP);
  Serial.print("FTP_PWD: ");
  Serial.println(pwd_FTP);
  Serial.println("\n-->Waiting for connections...");

  // Start FTP server
  ftpSrv.begin(user_FTP, pwd_FTP);
  ftpSrv.setLocalIp(WiFi.softAPIP()); // Must setLocalIP as the same as the one in softAPConfig()
  while (digitalRead(GPIO_SET_ACCESS_POINT) == LOW) {
    ftpSrv.handleFTP();
    delay(10);
  }
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
}

/**
 * Start LittleFS
 */
void mountLittleFS(){
  if (!LittleFS.begin()) {
    return; // LittleFS mount failed
  }
#ifdef DEBUG_MODE
  LittleFS.info(fs_info);
  Serial.print("-->LittleFS Mount OK\nTotal FS Size (kB):");
  Serial.println(fs_info.totalBytes*0.001);
  Serial.print("Used (kB): ");
  Serial.println(fs_info.usedBytes*0.001);
#endif // DEBUG_MODE
}

/**
 * Check what sensors are connected at first boot, and create files for each sensor
 * If sensor is not present at first boot, the file is not created, and following measurements are not taken
 * This prevents polling inactive of faulty sensors
 */
void createDataFiles(){
  File new_file;
  // Create Data files, if they do not exist
  if (!LittleFS.exists(sht30_1_file_path)){ // File does not exist
    new_file = LittleFS.open(sht30_1_file_path, "w");
    if (!new_file) {
      return; // Failed creating file
    }
    else{
      new_file.print("[SleepTime(s)=");
      new_file.print(SLEEP_TIME_MS/1000);
      new_file.print("]");
      new_file.print(sht30_csv_header_description); // Write CSV header
      new_file.close();
    }
  }
  if (!LittleFS.exists(sht30_2_file_path)){ // File does not exist
    new_file = LittleFS.open(sht30_2_file_path, "w");
    if (!new_file) {
      return; // Failed creating file
    }
    else{
      new_file.print("[SleepTime(s)=");
      new_file.print(SLEEP_TIME_MS/1000);
      new_file.print("]");
      new_file.print(sht30_csv_header_description); // Write CSV header
      new_file.close();
    }
  }
}

/**
 * Read and save sensor data on data files
 */
void dataAcquisition() {
  sht30_1_handler.cTemp = 0;
  sht30_2_handler.cTemp = 0;
  if (LittleFS.exists(sht30_1_file_path)){ // File exists
    if(sht30_1_handler.read_single_shot() == SHT30_READ_OK){
      appendSHT30Data(SHT30_I2C_ADDR_PIN_HIGH, sht30_1_handler.cTemp, sht30_1_handler.humidity, sht30_1_file_path); // Append sensor data
    }
  }
  if (LittleFS.exists(sht30_2_file_path)){ // File exists
    if(sht30_2_handler.read_single_shot() == SHT30_READ_OK){
      appendSHT30Data(SHT30_I2C_ADDR_PIN_LOW, sht30_2_handler.cTemp, sht30_2_handler.humidity, sht30_2_file_path); // Append sensor data
    }
  }
}

/**
 * Append the SHT30 data into the Data files
 */
void appendSHT30Data(uint8_t i2c_addr, float temp, float r_hum, const char *path) {
  File data_file = LittleFS.open(path, "a");
    if (!data_file) {
      return;
    }
    else {
      data_file.print("\n");
      data_file.print(cycle_counter);
      data_file.print(";");
      data_file.print(temp, 2);
      data_file.print(";");
      data_file.print(r_hum, 2);
      data_file.flush(); // Ensure writting before returning
    }
    data_file.close();
#ifdef DEBUG_MODE
    Serial.print("\n-->SHT30 ");
    Serial.print(i2c_addr, HEX);
    Serial.print(" T(C): ");
    Serial.print(temp);
    Serial.print(" RH(RH%): ");
    Serial.print(r_hum);
#endif // DEBUG_MODE
}

/**
 * Append the execution time (obtained by the return value of the millis() functions)
 */
void appendExecutionTime(int exe_time, const char *path) {

  File data_file = LittleFS.open(path, "a");
    if (!data_file) {
      return;
    }
    else {
      data_file.print(";");
      data_file.print(exe_time);
      data_file.flush(); // Ensure writting before returning
    }
    data_file.close();
}

void setup() {
  system_rtc_mem_read(64, &cycle_counter, 4); // Copy RTC memory value in current cycle counter
  // When reset by GPIO RTC memeory gets wrong value, this prevents errors in first boot if done by GPIO reset
  if (cycle_counter > CYCLE_COUNTER_RST){cycle_counter = 0;}
#ifdef DEBUG_MODE
  Serial.begin(SERIAL_SPEED);
  delay(500);
  Serial.println("\n\n*** SETUP ***");
  Serial.print("-->SW VERSION: ");
  Serial.println(APP_VERSION);
  Serial.print(F("Number of Cycles executed = "));
  Serial.println(cycle_counter);
  Serial.flush();
#endif // DEBUG_MODE
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED off
  pinMode(GPIO_SET_ACCESS_POINT, INPUT_PULLUP); // Set Access point pin
  Wire.begin(); // I2C
  WiFi.mode(WIFI_OFF); // Must turn the modem off; using disconnect won't work
  WiFi.forceSleepBegin();

  mountLittleFS();

  createDataFiles();

  dataAcquisition();
  
  // Increment and save the current cycle counter value in RTC memory
  cycle_counter++;
  system_rtc_mem_write(64, &cycle_counter, 4);

  if (digitalRead(GPIO_SET_ACCESS_POINT) == LOW){ // Change to AP mode
    digitalWrite(LED_BUILTIN, LOW); // LED on
    cycle_counter = 0;
    system_rtc_mem_write(64, &cycle_counter, 4); // Reset the cycle counter
    setAccessPoint();
  }

  int exe_time = millis();
  //if (sht30_1_handler.cTemp>0){appendExecutionTime(exe_time, sht30_1_file_path);}
  //if (sht30_2_handler.cTemp>0)appendExecutionTime(exe_time, sht30_2_file_path);

#ifdef DEBUG_MODE
  Serial.print("\n-->Execution time (ms): ");
  Serial.print(exe_time);
  Serial.print(F("\n-->Device will deep Sleep for (ms): "));
  Serial.println(SLEEP_TIME_MS);
  LittleFS.info(fs_info);
  Serial.print("-->LittleFS End of Execution\nTotal FS Size (kB):");
  Serial.println(fs_info.totalBytes*0.001);
  Serial.print("Used (kB): ");
  Serial.println(fs_info.usedBytes*0.001);
  Serial.flush();
#endif // DEBUG_MODE
  ESP.deepSleep(SLEEP_TIME_MS*1000, WAKE_NO_RFCAL); // Deep Sleep - MCU reset at wakeup - GPIO16 must be connected to RST
}


void loop() { }
