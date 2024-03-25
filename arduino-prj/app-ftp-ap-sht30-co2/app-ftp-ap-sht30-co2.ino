/*
 * app-ftp-ap-sht30-co2.ino
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
 *  Version 1.1 (Mar 25, 2024)
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
#define APP_VERSION "app-ftp-ap-sht30-co2 V1.1"

// Defines for the data aquisition system
#define SHT30_I2C_ADDR_PIN_HIGH 0x45  // Jumper NOT connected
#define SHT30_I2C_ADDR_PIN_LOW 0x44  // Jumper connected
#define SLEEP_TIME_MS 5000  // Interval between measurements in ms (time to execute the program ~1500ms)
#define GPIO_SET_ACCESS_POINT 14 // On Wemos D1 Mini - Pin number 14 (GPIO14)
#define GPIO_VIRTUAL_SERIAL_TX 12 // On Wemos D1 Mini - Pin number 12 (GPIO12)
#define GPIO_VIRTUAL_SERIAL_RX 13 // On Wemos D1 Mini - Pin number 13 (GPIO13)
#define VIRTUAL_SERIAL_SPEED  9600 // Serial speed for SprintIR-WF-100 sensor
#define SPRINTIR_MAX_POLL_REPS  5 // Maximum number of atempts to read a correct value from the CO2 sensor (first 1~2 readings return 0)
#define SPRINTIR_MIN_READ_VALUE  1 // Minimum value returned for a correct reading

// Remove when deploying in production environment
#define DEBUG_MODE

#define CYCLE_COUNTER_RST 10000000  // When resetig via GPIO, RTC memory gets wrong counter value - must be reset
unsigned long cycle_counter = 0;

// CO2 sensor
SoftwareSerial virtual_serial(GPIO_VIRTUAL_SERIAL_RX, GPIO_VIRTUAL_SERIAL_TX);
COZIR czr_handler(&virtual_serial);
int sprintir_co2_status = 0;
     
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
// Data file configuration
const char* sht30_1_file_path = "/sht30_addr_45_data.csv";
const char* sht30_2_file_path = "/sht30_addr_44_data.csv";
const char* sht30_csv_header_description = "Counter;Temperature(C);RelHumidity(RH%);ExecutionTime(ms)"; // Added at the creation of the Data file
const char* sprintir_co2_file_path = "/sprintir_co2_data.csv";
const char* sprintir_co2_csv_header_description = "Counter;CO2Level(ppm);ExecutionTime(ms)"; // Added at the creation of the Data file


/** PF definitions **/

/**
 * SPRINTIR-W-F-100 CO2 sensor polling mode (runs a number of readings on the sensor, returns when valid reading is detected)
 */
int sprintirPoll(int reps, int calibrate){
  int co2_ppm=0, k=0;
  czr_handler.init(); // Initialize CO2 sensor handler (Setup time is set to 1200 ms)
  virtual_serial.begin(VIRTUAL_SERIAL_SPEED);
  //czr_handler.setOperatingMode(CZR_POLLING); // Explicit set of Polling mode

  while(co2_ppm<SPRINTIR_MIN_READ_VALUE && k<reps){
      co2_ppm = czr_handler.CO2();
      delay(1000);
      k++;
#ifdef DEBUG_MODE
      Serial.print("\n-->POLLING CO2(ppm)=\t");
      Serial.println(co2_ppm);
#endif // DEBUG_MODE
    }
  co2_ppm *= czr_handler.getPPMFactor();  // In Sprintir WF 100, the correction factor is 100
  czr_handler.setOperatingMode(CZR_COMMAND); // Explicit set of Command mode (this reduces energy use when in Sleep Mode)
  virtual_serial.end();
  return co2_ppm;
}

/**
 * Configures the ESP8266 as WiFi Access Point, and starts the FTP server
 */
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
  if (!LittleFS.exists(sprintir_co2_file_path)){ // File does not exist
    new_file = LittleFS.open(sprintir_co2_file_path, "w");
    if (!new_file) {
      return; // Failed creating file
    }
    else{
      new_file.print("[SleepTime(s)=");
      new_file.print(SLEEP_TIME_MS/1000);
      new_file.print("]");
      new_file.print(sprintir_co2_csv_header_description); // Write CSV header
      new_file.close();
    }
  }
}

/**
 * Read and save sensor data on data files
 */
void dataAcquisition() {
  int co2_ppm = 0;

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
  if (LittleFS.exists(sprintir_co2_file_path)){ // File exists
    co2_ppm = sprintirPoll(SPRINTIR_MAX_POLL_REPS, 1); // Enable calibration
    if (co2_ppm >= SPRINTIR_MIN_READ_VALUE){
      appendSprintirCO2Data(co2_ppm, sprintir_co2_file_path);
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
 * Append the CO2 data into the Data file
 */
void appendSprintirCO2Data(int co2_ppm, const char *path) {
  File data_file = LittleFS.open(path, "a");
    if (!data_file) {
      return;
    }
    else {
      data_file.print("\n");
      data_file.print(cycle_counter);
      data_file.print(";");
      data_file.print(co2_ppm);
      data_file.flush(); // Ensure writting before returning
    }
    data_file.close();
#ifdef DEBUG_MODE
    Serial.print("\n-->CO2(ppm)=\t");
    Serial.println(co2_ppm);
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
  appendExecutionTime(exe_time, sht30_1_file_path);
  appendExecutionTime(exe_time, sht30_2_file_path);
  appendExecutionTime(exe_time, sprintir_co2_file_path);

#ifdef DEBUG_MODE
  Serial.print("\n-->Execution time (ms): ");
  Serial.print(exe_time);
  Serial.print(F("\n-->Device will deep Sleep for (ms): "));
  Serial.println(SLEEP_TIME_MS);
  Serial.flush();
#endif // DEBUG_MODE
  ESP.deepSleep(SLEEP_TIME_MS*1000, WAKE_NO_RFCAL); // Deep Sleep - MCU reset at wakeup - GPIO16 must be connected to RST
}


void loop() { }
