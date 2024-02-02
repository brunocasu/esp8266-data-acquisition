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
// #include <ESP8266FtpServer.h>
#include <SimpleFTPServer.h> // Replacing <ESP8266FtpServer.h>

#define SERIAL_SPEED  115200

// Defines for the data aquisition system
#define SHT30_I2C_ADDR_PIN_HIGH 0x45  // Jumper NOT connected
#define SHT30_I2C_ADDR_PIN_LOW 0x44  // Jumper connected
#define SLEEP_TIME_MS 300000  // Interval between measurements in ms
#define GPIO_SET_ACCESS_POINT D5 // On Wemos D1 Mini - Pin number 14 (GPIO14)

// Remove when deploying in production environment
//#define DEBUG_MODE

// SHT30 configuration
class SHT3X{
public:
	SHT3X(uint8_t address=0x45);
	byte get(void);
	float cTemp=0;
	float fTemp=0;
	float humidity=0;

private:
	uint8_t _address;

};
SHT3X::SHT3X(uint8_t address)
{
	Wire.begin();
	_address=address;
}

byte SHT3X::get()
{
	unsigned int data[6];

	// Start I2C Transmission
	Wire.beginTransmission(_address);
	// Send measurement command
	Wire.write(0x2C);
	Wire.write(0x06);
	// Stop I2C transmission
	if (Wire.endTransmission()!=0)
		return 1;

	delay(1); // reduced delay betey master transmit and receive (500 ms is to much)

	// Request 6 bytes of data
	Wire.requestFrom(_address, 6);

	// Read 6 bytes of data
	// cTemp msb, cTemp lsb, cTemp crc, humidity msb, humidity lsb, humidity crc
	for (int i=0;i<6;i++) {
		data[i]=Wire.read();
	};

	delay(1);

	if (Wire.available()!=0)
		return 2;

	// Convert the data
	cTemp = ((((data[0] * 256.0) + data[1]) * 175) / 65535.0) - 45;
	fTemp = (cTemp * 1.8) + 32;
	humidity = ((((data[3] * 256.0) + data[4]) * 100) / 65535.0);

	return 0;
}
SHT3X sht30_1_handler(SHT30_I2C_ADDR_PIN_HIGH); // Addr 0x45
SHT3X sht30_2_handler(SHT30_I2C_ADDR_PIN_LOW); // Addr 0x44


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
const char* csv_header_description = "Temperature(C);RelHumidity(RH%)"; // Added at the creation of the Data file


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
  if(sht30_1_handler.get()==0){
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
  if(sht30_2_handler.get()==0){
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
}


void setup() {
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
  
  initFS(); // Start File system
  readSensors(); // Read sensor data and save on FS

  if (digitalRead(GPIO_SET_ACCESS_POINT) == LOW){ // Change to AP mode
    digitalWrite(LED_BUILTIN, LOW); // LED on
    setAccessPoint();
  }
  
#ifdef DEBUG_MODE
  Serial.print(F("\n-->Device Sleep for (ms): "));
  Serial.println(SLEEP_TIME_MS);
  Serial.print(F("Time since start of execution (ms) = "));
  Serial.println(millis());
  Serial.flush();
#endif // DEBUG_MODE

  ESP.deepSleep(SLEEP_TIME_MS*1000, WAKE_NO_RFCAL); // Deep Sleep - MCU reset at wakeup - GPIO16 must be connected to RST
}


void loop() { }
