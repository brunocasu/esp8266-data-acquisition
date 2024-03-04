/*
 * monitor-ftp-ap-deep-sleep.ino
 *
 *  Created on: Jan 25, 2024
 *      Author: Bruno Casu
 */
 
#include <ESP8266WiFi.h>
#include <coredecls.h>  // crc32()
#include <PolledTimeout.h>
#include <include/WiFiState.h>  // WiFiState structure details
#include <LittleFS.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SimpleFTPServer.h> // Replacing <ESP8266FtpServer.h>
#include <cozir.h>
#include <sht30.h>


#define SERIAL_SPEED  115200

// Defines for the data aquisition system
#define SHT30_I2C_ADDR_PIN_HIGH 0x45  // Jumper NOT connected
#define SHT30_I2C_ADDR_PIN_LOW 0x44  // Jumper connected
#define SLEEP_TIME_MS 5000  // Interval between measurements in ms
#define GPIO_SET_ACCESS_POINT 14 // On Wemos D1 Mini - Pin number 14 (GPIO14)
#define GPIO_VIRTUAL_SERIAL_TX 12 // On Wemos D1 Mini - Pin number 12 (GPIO12)
#define GPIO_VIRTUAL_SERIAL_RX 13 // On Wemos D1 Mini - Pin number 13 (GPIO13)

// Remove when deploying in production environment
#define DEBUG_MODE

// CO2 sensor
SoftwareSerial virtual_serial(GPIO_VIRTUAL_SERIAL_RX, GPIO_VIRTUAL_SERIAL_TX);
COZIR czr_handler(&virtual_serial);
     
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
const char* csv_header_description = "Timestamp(ms);Temperature(C);RelHumidity(RH%)"; // Added at the creation of the Data file
unsigned long cycle_counter = 0;

/** PF definitions **/


/**
 * Configures the ESP8266 as WiFi Access Point, and starts the FTP server
 */
void setAccessPoint() {
  // Configure WiFi Access point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid_AP); // No password for connecting
  WiFi.softAPConfig(local_IP, gateway, subnet_mask);
#ifdef DEBUG_MODE
  Serial.print("\n-->WiFi SSID: AP-ESP8266 (FTP Server IP Addr): ");
  Serial.println(WiFi.softAPIP());
  Serial.println("Waiting for connections...");
#endif // DEBUG_MODE
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
 * Mount the file system (LittleFS)
 * If the Data files are not created (first boot), create the Data files
 */
void initFS() {
  File new_file;
#ifdef DEBUG_MODE
  Serial.println("\n-->LittleFS start");
  LittleFS.format(); // WARNING When in DEBUG mode the FS is formated at reset
#endif // DEBUG_MODE
  if (!LittleFS.begin()) {
    return; // LittleFS mount failed
  }
#ifdef DEBUG_MODE
  LittleFS.info(fs_info);
  Serial.print("LittleFS Mount OK\nTotal FS Size (kB):");
  Serial.println(fs_info.totalBytes*0.001);
  Serial.print("Used (kB): ");
  Serial.println(fs_info.usedBytes*0.001);
#endif // DEBUG_MODE

  // Create Data Files (if they do not exist)
  if (readFile(sht30_1_file_path) == 0){ // File does not exist
    new_file = LittleFS.open(sht30_1_file_path, "w"); // Create file
    if (!new_file) {
      return; // Failed creating file
    }
    else {
      new_file.print(csv_header_description); // Write CSV header
      new_file.close();
    }
  }
  if (readFile(sht30_2_file_path) == 0){ // File does not exist
    new_file = LittleFS.open(sht30_2_file_path, "w"); // Create file
    if (!new_file) {
      return; // Failed creating file
    }
    else {
      new_file.print(csv_header_description); // Write CSV header
      new_file.close();
    }
  }
}


/**
 * Append the sensor data into the Data files
 * Include internal VCC reading on the data file
 */
void appendDataFile(float temp, float r_hum, const char *path) {
  int exe_time = millis();
  int timestamp_ms = (cycle_counter*SLEEP_TIME_MS) + exe_time;
  File data_file = LittleFS.open(path, "a");
    if (!data_file) {
      return;
    }
    else {
      data_file.print("\n");
      data_file.print(temp, 2);
      data_file.print(";");
      data_file.print(r_hum, 2);
      data_file.flush(); // Ensure writting before returning
    }
    data_file.close();
}


/**
 * Read file from the FS
 */
int readFile(const char *path) {
  File file = LittleFS.open(path, "r");
  if (!file) {
    return 0; // File does not exist
  }
  else {
    file.close();
    return 1; // File already exist
  }
}


/**
 * Read both sensors and write the returned values on the corresponding data file
 * If sensor read fails set sensor to SENSOR_ERROR status
 */
void readSensors() {
  uint32_t cozir_ctr = 0;
  uint32_t co2_ppm = 0;
  // SHT30
  if(sht30_1_handler.read_single_shot() == SHT30_READ_OK){
    appendDataFile(sht30_1_handler.cTemp, sht30_1_handler.humidity, sht30_1_file_path);
#ifdef DEBUG_MODE
    Serial.print("\n-->Sensor ADDR");
    Serial.println(SHT30_I2C_ADDR_PIN_HIGH, HEX);
    Serial.print("Temperature (C): ");
    Serial.println(sht30_1_handler.cTemp);
    Serial.print("Relative Humidity (RH%): ");
    Serial.println(sht30_1_handler.humidity);
#endif // DEBUG_MODE
  }
  // Repeat for next sensor
  if(sht30_2_handler.read_single_shot() == SHT30_READ_OK){
    appendDataFile(sht30_2_handler.cTemp, sht30_2_handler.humidity, sht30_2_file_path);
#ifdef DEBUG_MODE
    Serial.print("\n-->Sensor ADDR");
    Serial.println(SHT30_I2C_ADDR_PIN_LOW, HEX);
    Serial.print("Temperature (C): ");
    Serial.println(sht30_2_handler.cTemp);
    Serial.print("Relative Humidity (RH%): ");
    Serial.println(sht30_2_handler.humidity);
#endif // DEBUG_MODE
  }
  
  // CO2 Sensor
  // The GSS CO2 sensor does not provide an accurate measurement on the first attempts, must be polled (up to 5x)
#ifdef DEBUG_MODE
  Serial.print("-->GSS SPRINTIR VERSION: ");  
#endif // DEBUG_MODE
  delay(100);
  czr_handler.getVersionSerial();
  delay(5);
  while (virtual_serial.available())
  {
    Serial.write(virtual_serial.read());
  }
  delay(100);
  
  while (co2_ppm<100 && cozir_ctr<5){
    //delay(100);
    co2_ppm = czr_handler.CO2();
    co2_ppm *= czr_handler.getPPMFactor();  // most of time PPM = one.
    cozir_ctr++;
  }
#ifdef DEBUG_MODE
  Serial.print("\n-->CO2(ppm)=\t");
  Serial.println(co2_ppm);    
#endif // DEBUG_MODE

}


void setup() {
  uint32_t cozir_ctr = 0;
  uint32_t co2_ppm = 0;
#ifdef DEBUG_MODE
  Serial.begin(SERIAL_SPEED);
  delay(10);
  Serial.println("\n***ESP8266 Temperature Monitor***");
#endif // DEBUG_MODE
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED off
  pinMode(GPIO_SET_ACCESS_POINT, INPUT_PULLUP); // Set Access point pin
  Wire.begin(); // I2C
  WiFi.mode(WIFI_OFF); // Must turn the modem off; using disconnect won't work
  WiFi.forceSleepBegin();

  virtual_serial.begin(9600);
  czr_handler.init();

  system_rtc_mem_read(64, &cycle_counter, 4);
  
  initFS(); // Start File system
  readSensors(); // Read sensor data and save on FS

  sht30_2_handler.read_status_register();
  Serial.print("Status Register SHT30 45: ");
  Serial.println(sht30_2_handler.status_reg, HEX);

  if (digitalRead(GPIO_SET_ACCESS_POINT) == LOW){ // Change to AP mode
    digitalWrite(LED_BUILTIN, LOW); // LED on
    cycle_counter = 0;
    system_rtc_mem_write(64, &cycle_counter, 4); // Reset the cycle counter
    setAccessPoint();
  }

  cycle_counter++;
  system_rtc_mem_write(64, &cycle_counter, 4);
  
#ifdef DEBUG_MODE
  Serial.print(F("\n-->Device will deep Sleep for (ms): "));
  Serial.println(SLEEP_TIME_MS);
  Serial.print(F("Time since start of execution (ms) = "));
  Serial.println(millis());
  Serial.print(F("Number of Cycles executed = "));
  Serial.println(cycle_counter);
  Serial.flush();
#endif // DEBUG_MODE

  ESP.deepSleep(SLEEP_TIME_MS*1000, WAKE_NO_RFCAL); // Deep Sleep - MCU reset at wakeup - GPIO16 must be connected to RST
}


void loop() { }
