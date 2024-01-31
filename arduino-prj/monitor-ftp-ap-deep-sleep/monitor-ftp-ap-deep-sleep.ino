/*
 * monitor-ftp-ap-deep-sleep.ino
 *
 *  Created on: Jan 25, 2024
 *      Author: Bruno Casu
 */
 
#include <WEMOS_SHT3X.h>
#include <ESP8266WiFi.h>
#include <coredecls.h>  // crc32()
#include <PolledTimeout.h>
#include <include/WiFiState.h>  // WiFiState structure details
#include <LittleFS.h>
//#include <ESP8266FtpServer.h>
#include <Wire.h>
#include <SimpleFTPServer.h>

#define SERIAL_SPEED  115200

// Defines for the data aquisition system
#define SHT30_I2C_ADDR_PIN_HIGH 0x45  // Jumper NOT connected
#define SHT30_I2C_ADDR_PIN_LOW 0x44  // Jumper connected
#define SLEEP_TIME_MS 5000  // Interval between measurements
#define GPIO_SET_ACCESS_POINT D5 // On Wemos D1 Mini - Pin number 14 (GPIO14)

// Remove when deploying in production environment
#define DEBUG_MODE

typedef enum {
  BLUE_LED_ON,
  BLUE_LED_OFF
} led_status_t;

typedef enum {
  PROCESS_DATA_AQUISITION,
  PROCESS_ACCESS_POINT
} process_status_t;

typedef enum {
  SENSOR_UNINITIALIZED,
  SENSOR_OK,
  SENSOR_ERROR
} sensor_status_t;

process_status_t process_status = PROCESS_DATA_AQUISITION;

// SHT30 configuration
SHT3X sht30_1_handler(SHT30_I2C_ADDR_PIN_HIGH);
SHT3X sht30_2_handler(SHT30_I2C_ADDR_PIN_LOW);
sensor_status_t sht30_1_status = SENSOR_UNINITIALIZED;
sensor_status_t sht30_2_status = SENSOR_UNINITIALIZED;

// WiFi Access Point configuration
IPAddress local_IP(192,168,4,22);
IPAddress gateway(192,168,4,9);
IPAddress subnet_mask(255,255,255,0);
const char* ssid_AP = "AP-ESP8266";

// FTP configuration
const char* user_FTP = "esp8266";
const char* pwd_FTP = "esp8266";
FtpServer ftpSrv;

// Data file configuration
//File sht30_1_DataFile;
//File sht30_1_DataFile;
const char* sht30_1_file_path = "/sht30_addr_45_data.csv";
const char* sht30_2_file_path = "/sht30_addr_44_data.csv";

// ADC_MODE(ADC_VCC);

void setup() {
  // Init peripherals and GPIOs
  WiFi.mode(WIFI_OFF); // Must turn the modem off; using disconnect won't work
  WiFi.forceSleepBegin();
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  pinMode(GPIO_SET_ACCESS_POINT, INPUT_PULLUP);
  Wire.begin();
#ifdef DEBUG_MODE
  Serial.begin(SERIAL_SPEED);
  delay(200);
  Serial.println("\n***ESP8266 Temperature Monitor***");
  float result = ESP.getVcc();
  Serial.print("Input VCC (V): ");
  Serial.println((double) result * 0.001);
#endif // DEBUG_MODE
  // Test SHT30 sensors
  sensorsInit();
  // Start File system
  initFS();
}

void loop() {
  // Check if pin 14 is set to GND
  if (digitalRead(GPIO_SET_ACCESS_POINT) == LOW){
    // Change system to WiFi Access point for downloading the data file
    process_status = PROCESS_ACCESS_POINT;
  }
  else{
    process_status = PROCESS_DATA_AQUISITION;
  }

  if (process_status == PROCESS_DATA_AQUISITION){
    setLED(BLUE_LED_ON);
    delay(50);
    setLED(BLUE_LED_OFF);
    readSensors();
    deviceSleep(SLEEP_TIME_MS);
    listDir("/");
    readFile(sht30_1_file_path);
    delay(1000);
  }
  else if (process_status == PROCESS_ACCESS_POINT){
    setLED(BLUE_LED_ON);
    setAccessPoint();
  }
}
/** PF definitions **/

void sensorsInit() {
  if(sht30_1_handler.get()==0){
    sht30_1_status = SENSOR_OK;
  }
  else{
    sht30_1_status = SENSOR_ERROR;
  }

  if(sht30_2_handler.get()==0){
    sht30_2_status = SENSOR_OK;
  }
  else{
    sht30_2_status = SENSOR_ERROR;
  }
}

void readSensors() {
  if(sht30_1_handler.get()==0 && sht30_1_status == SENSOR_OK){
#ifdef DEBUG_MODE
    Serial.print("\n-->Sensor ADDR");
    Serial.println(SHT30_I2C_ADDR_PIN_HIGH, HEX);
    Serial.print("Temperature (C): ");
    Serial.println(sht30_1_handler.cTemp);
    Serial.print("Relative Humidity (RH%): ");
    Serial.println(sht30_1_handler.humidity);
#endif // DEBUG_MODE
    // Write SHT30 data into Data File
    appendDataFile(sht30_1_handler.cTemp, sht30_1_handler.humidity, sht30_1_file_path);
  }
  else{
#ifdef DEBUG_MODE
    Serial.print("\n-->SENSOR_ERROR - ADDR");
    Serial.println(SHT30_I2C_ADDR_PIN_HIGH, HEX);
#endif // DEBUG_MODE
    sht30_1_status = SENSOR_ERROR;
  }

  if(sht30_2_handler.get()==0 && sht30_2_status == SENSOR_OK){
#ifdef DEBUG_MODE
    Serial.print("\n-->Sensor ADDR");
    Serial.println(SHT30_I2C_ADDR_PIN_LOW, HEX);
    Serial.print("Temperature (C): ");
    Serial.println(sht30_2_handler.cTemp);
    Serial.print("Relative Humidity (RH%): ");
    Serial.println(sht30_2_handler.humidity);
#endif // DEBUG_MODE
    // Write SHT30 data into Data File
    appendDataFile(sht30_2_handler.cTemp, sht30_2_handler.humidity, sht30_2_file_path);
  }
  else{
#ifdef DEBUG_MODE
    Serial.print("\n-->SENSOR_ERROR - ADDR");
    Serial.println(SHT30_I2C_ADDR_PIN_LOW, HEX);
#endif // DEBUG_MODE
    sht30_2_status = SENSOR_ERROR;
  }
}

void setAccessPoint() {
  // Configure WiFi Access point
  WiFi.softAPConfig(local_IP, gateway, subnet_mask);
  WiFi.softAP(ssid_AP); // No password for connecting
#ifdef DEBUG_MODE
  Serial.print("\n-->WiFi SSID: AP-ESP8266 (IP Addr): ");
  Serial.println(WiFi.softAPIP());

  // Start FTP server
  Serial.println("FTP Server\nWaiting for connections...");
#endif // DEBUG_MODE
  ftpSrv.begin(user_FTP, pwd_FTP);
  ftpSrv.setLocalIp(WiFi.softAPIP());
  while (digitalRead(GPIO_SET_ACCESS_POINT) == LOW) {
    ftpSrv.handleFTP();
    delay(10);
  }
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
}

void initFS() {
  File new_file;
#ifdef DEBUG_MODE
  Serial.println("\n-->LittleFS start");
  LittleFS.format();
#endif // DEBUG_MODE
  if (!LittleFS.begin()) {
    // LittleFS mount failed
    Serial.println("\n-->LittleFS mount failed");
    return;
  }
  else{
    // Create file for SHT30 Addr 45 data // TODO remove seral prints
    new_file = LittleFS.open(sht30_1_file_path, "w");
    if (!new_file) {
      Serial.println("Failed to open file for writing");
      return;
    }
    if (new_file.print("Temperature(C);RelHumidity(RH%)")) {
      Serial.println("File written");
    } else {
      Serial.println("Write failed");
    }
    new_file.close();
  }
}

void appendDataFile(float temp, float r_hum, const char *path) {
  //sht30_1_DataFile = LittleFS.open(sht30_1_file_path, "a");
  File data_file = LittleFS.open(path, "a");
    if (!data_file) {
      Serial.print("Unable to open file");
      Serial.println(path);
    } else {
      data_file.print("\n");
      data_file.print(temp, 2);
      data_file.print(";");
      data_file.print(r_hum, 2);
      //data_file.flush();
    }
    data_file.close();
}

void readFile(const char *path) {
  Serial.printf("\nReading file: %s\n", path);

  File file = LittleFS.open(path, "r");
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file:\n");
  while (file.available()) { Serial.write(file.read()); }
  file.close();
}

void listDir(const char *dirname) {
  Serial.printf("\nListing directory: %s\n", dirname);

  Dir root = LittleFS.openDir(dirname);

  while (root.next()) {
    File file = root.openFile("r");
    Serial.print("  FILE: ");
    Serial.print(root.fileName());
    Serial.print("  SIZE: ");
    Serial.print(file.size());
    file.close();
  }
}

void deviceSleep(int sleep_time) {
#ifdef DEBUG_MODE
  Serial.print(F("\n-->Device Sleep for (ms): "));
  Serial.println(sleep_time);
  delay(sleep_time);
#else
  // TODO add deep sleep function
#endif // DEBUG_MODE
}

void setLED(led_status_t cmd) {
  if(cmd == BLUE_LED_ON){
    digitalWrite(LED_BUILTIN, LOW);
  }
  else if(cmd == BLUE_LED_OFF){
    digitalWrite(LED_BUILTIN, HIGH);
  }
}
