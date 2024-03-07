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
#include <Wire.h>
// #include <ESP8266FtpServer.h>
#include <SimpleFTPServer.h> // Replacing <ESP8266FtpServer.h>

#define SERIAL_SPEED  115200

// Defines for the data aquisition system
#define SHT30_I2C_ADDR_PIN_HIGH 0x45  // Jumper NOT connected
#define SHT30_I2C_ADDR_PIN_LOW 0x44  // Jumper connected
#define SLEEP_TIME_MS 5000  // Interval between measurements in ms
#define GPIO_SET_ACCESS_POINT D5 // On Wemos D1 Mini - Pin number 14 (GPIO14)

// Remove when deploying in production environment
// #define DEBUG_MODE

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
SHT3X sht30_1_handler(SHT30_I2C_ADDR_PIN_HIGH); // Addr 0x45
SHT3X sht30_2_handler(SHT30_I2C_ADDR_PIN_LOW); // Addr 0x44
sensor_status_t sht30_1_status = SENSOR_UNINITIALIZED;
sensor_status_t sht30_2_status = SENSOR_UNINITIALIZED;

// WiFi Access Point configuration
IPAddress local_IP(10,10,10,1);
IPAddress gateway(10,10,10,1);
IPAddress subnet_mask(255,255,255,0);
const char* ssid_AP = "AP-ESP8266";

// FTP configuration
const char* user_FTP = "esp8266";
const char* pwd_FTP = "esp8266";
FtpServer ftpSrv; // Handler

// Data file configuration
const char* sht30_1_file_path = "/sht30_addr_45_data.csv";
const char* sht30_2_file_path = "/sht30_addr_44_data.csv";
const char* csv_header_description = "Timestamp(s);Temperature(C);RelHumidity(RH%);VCC(V)"; // Added at the creation of the Data file

unsigned long time_ms;

ADC_MODE(ADC_VCC);

/** PF definitions **/


/**
 * Initialize both sensors with an inital I2C transaction
 * If sensor not connect or faulty, set sensor status to SENSOR_ERROR
 */
void initSensors() {
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


/**
 * Read both sensors and write the returned values on the corresponding data file
 * If sensor read fails set sensor to SENSOR_ERROR status
 */
void readSensors() {
  time_ms = millis();
  if(sht30_1_handler.get()==0 && sht30_1_status == SENSOR_OK){
#ifdef DEBUG_MODE
    Serial.print("\n-->Sensor ADDR");
    Serial.println(SHT30_I2C_ADDR_PIN_HIGH, HEX);
    Serial.print("Temperature (C): ");
    Serial.println(sht30_1_handler.cTemp);
    Serial.print("Relative Humidity (RH%): ");
    Serial.println(sht30_1_handler.humidity);
#endif // DEBUG_MODE
    appendDataFile(sht30_1_handler.cTemp, sht30_1_handler.humidity, sht30_1_file_path);
  }
  else{
#ifdef DEBUG_MODE
    Serial.print("\n-->SENSOR_ERROR - ADDR");
    Serial.println(SHT30_I2C_ADDR_PIN_HIGH, HEX);
#endif // DEBUG_MODE
    sht30_1_status = SENSOR_ERROR;
  }
  // Repeat for next sensor
  time_ms = millis();
  if(sht30_2_handler.get()==0 && sht30_2_status == SENSOR_OK){
#ifdef DEBUG_MODE
    Serial.print("\n-->Sensor ADDR");
    Serial.println(SHT30_I2C_ADDR_PIN_LOW, HEX);
    Serial.print("Temperature (C): ");
    Serial.println(sht30_2_handler.cTemp);
    Serial.print("Relative Humidity (RH%): ");
    Serial.println(sht30_2_handler.humidity);
#endif // DEBUG_MODE
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
  Serial.print("LittleFS Mount OK - FS Size (B):");
  Serial.println(fs_info.totalBytes);
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
  float vcc_reading = ESP.getVcc();
  float time_s = time_ms*0.001; // Time of execution is read at the start of reading the sensor data
  File data_file = LittleFS.open(path, "a");
    if (!data_file) {
      Serial.print("Unable to open file");
      Serial.println(path);
    }
    else {
      data_file.print("\n");
      data_file.print(time_s, 3);
      data_file.print(";");
      data_file.print(temp, 2);
      data_file.print(";");
      data_file.print(r_hum, 2);
      data_file.print(";");
      data_file.print(vcc_reading*0.001, 2);
      data_file.flush();
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
#ifdef DEBUG_MODE
    Serial.printf("\nReading file: %s\n", path);
    Serial.print("Content:\n");
    while (file.available()) { Serial.write(file.read()); }
#endif // DEBUG_MODE
    file.close();
    return 1;
  }
}


/**
 * List the files in the selected directory in the FS
 * Only used for DEBUG
 */
void listDir(const char *dirname) {
  Dir root = LittleFS.openDir(dirname);
  Serial.printf("\nListing directory: %s\n", dirname);
  while (root.next()) {
    File file = root.openFile("r");
    Serial.print("  FILE: ");
    Serial.print(root.fileName());
    Serial.print("  SIZE: ");
    Serial.print(file.size());
    file.close();
  }
}


/**
 * Set the device in sleep mode (NOP)
 */
void deviceSleep(int sleep_time) {
#ifdef DEBUG_MODE
  Serial.print(F("\n-->Device Sleep for (ms): "));
  Serial.println(sleep_time);
  delay(sleep_time);
#else
  delay(sleep_time); // TODO add deep sleep function
#endif // DEBUG_MODE
}


/**
 * Set the LED status
 */
void setLED(led_status_t cmd) {
  if(cmd == BLUE_LED_ON){
    digitalWrite(LED_BUILTIN, LOW);
  }
  else if(cmd == BLUE_LED_OFF){
    digitalWrite(LED_BUILTIN, HIGH);
  }
}


void setup() {
  WiFi.mode(WIFI_OFF); // Must turn the modem off; using disconnect won't work
  WiFi.forceSleepBegin();
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  pinMode(GPIO_SET_ACCESS_POINT, INPUT_PULLUP);
  Wire.begin();
#ifdef DEBUG_MODE
  Serial.begin(SERIAL_SPEED);
  delay(200);
  Serial.println("\n***ESP8266 Temperature Monitor***");
#endif // DEBUG_MODE
  initSensors(); // Test SHT30 sensors
  initFS(); // Start File system
}


void loop() {
  if (digitalRead(GPIO_SET_ACCESS_POINT) == LOW){
    process_status = PROCESS_ACCESS_POINT;
  }
  else{
    process_status = PROCESS_DATA_AQUISITION;
  }

  if (process_status == PROCESS_DATA_AQUISITION){
    setLED(BLUE_LED_ON);
    delay(10);
    setLED(BLUE_LED_OFF);
    readSensors();
    deviceSleep(SLEEP_TIME_MS);
  }
  else if (process_status == PROCESS_ACCESS_POINT){
    setLED(BLUE_LED_ON);
    setAccessPoint();
  }
}
