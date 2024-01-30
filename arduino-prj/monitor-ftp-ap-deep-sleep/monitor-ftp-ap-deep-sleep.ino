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
#include <ESP8266FtpServer.h>
#include <Wire.h>

#define SERIAL_SPEED  115200

// defines for the data aquisition system
#define SHT30_I2C_ADDR_PIN_HIGH 0x45
#define SHT30_I2C_ADDR_PIN_LOW 0x44
#define SLEEP_TIME_MS 5000
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

// SHT30 Shield
SHT3X sht30_1_handler(SHT30_I2C_ADDR_PIN_HIGH);
sensor_status_t sht30_1_status = SENSOR_UNINITIALIZED;

// SHT30 connect via cable in I2C port
SHT3X sht30_2_handler(SHT30_I2C_ADDR_PIN_LOW);
sensor_status_t sht30_2_status = SENSOR_UNINITIALIZED;

// Access Point configuration
IPAddress local_IP(192,168,4,22);
IPAddress gateway(192,168,4,9);
IPAddress subnet_mask(255,255,255,0);
const char* ssid_AP = "AP-ESP8266";
const char* user_FTP = "esp8266";
const char* pwd_FTP = "esp8266";
FtpServer ftpSrv;

// Data file configuration
File measurementsDataFile;
const char* measurementsDataFileName = "esp8266Data.txt";

ADC_MODE(ADC_VCC);

void setup() {
  // Init peripherals and GPIOs
  WiFi.mode(WIFI_OFF); // Must turn the modem off; using disconnect won't work
  WiFi.forceSleepBegin();
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  pinMode(GPIO_SET_ACCESS_POINT, INPUT_PULLUP);
  Serial.begin(SERIAL_SPEED);
  Wire.begin();
  
#ifdef DEBUG_MODE
  delay(200);
  Serial.println("\n***ESP8266 Temperature Monitor***");
  float result = ESP.getVcc();
  Serial.print("VCC (V): ");
  Serial.println((double) result * 0.001);
#endif // DEBUG_MODE
  // Test SHT30 sensors
  sensorsInit();
  // Start File system
  if (!LittleFS.begin()) {
#ifdef DEBUG_MODE
    Serial.println("\n-->LittleFS mount failed");
#endif // DEBUG_MODE
    return;
  }
  else{
    Serial.println("\n-->LittleFS - Create Data File");
    createDataFile();
  }
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
  }
  else if (process_status == PROCESS_ACCESS_POINT){
    setLED(BLUE_LED_ON);
    setAccessPoint();
  }
}

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
    appendDataFile(sht30_1_handler.cTemp, sht30_1_handler.humidity, SHT30_I2C_ADDR_PIN_HIGH);
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
    appendDataFile(sht30_2_handler.cTemp, sht30_2_handler.humidity, SHT30_I2C_ADDR_PIN_LOW);
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
  Serial.print("WiFi SSID: AP-ESP8266 (IP Addr): ");
  Serial.println(WiFi.softAPIP());

  // Start FTP server
  Serial.println("FTP Server\nWaiting for connections...");
#endif // DEBUG_MODE
  ftpSrv.begin(user_FTP, pwd_FTP);
  while (digitalRead(GPIO_SET_ACCESS_POINT) == LOW) {
    ftpSrv.handleFTP();
    delay(10);
  }
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
}

void createDataFile(){
  if (LittleFS.exists(measurementsDataFileName)) {
      Serial.print("Result file reported existing: ");
      Serial.println(measurementsDataFileName);
      measurementsDataFile = LittleFS.open(measurementsDataFileName, "r");
      if (!measurementsDataFile) {
        Serial.println("Unable to open result file for reading.");
      } 
      else {
        Serial.print("File reported to be ");
        Serial.print(measurementsDataFile.size());
        Serial.println(" bytes in size.");
      }
  }
  else {
    Serial.print("Result file reported NOT existing. Proceeding to create a new one: ");
    Serial.println(measurementsDataFileName);
  }
}

void appendDataFile(float temp, float r_hum, char i2c_addr){
  measurementsDataFile = LittleFS.open(measurementsDataFileName, "a");
    if (!measurementsDataFile) {
      Serial.println("Unable to open result file for appending.");
    } else {
      measurementsDataFile.print(temp, 2);
      measurementsDataFile.print(";");
      measurementsDataFile.print(r_hum, 2);
      measurementsDataFile.print(";");
      measurementsDataFile.print(i2c_addr, HEX);
      measurementsDataFile.flush();
    }
    measurementsDataFile.close();
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

void setLED(led_status_t cmd){
  if(cmd == BLUE_LED_ON){
    digitalWrite(LED_BUILTIN, LOW);
  }
  else if(cmd == BLUE_LED_OFF){
    digitalWrite(LED_BUILTIN, HIGH);
  }
}
