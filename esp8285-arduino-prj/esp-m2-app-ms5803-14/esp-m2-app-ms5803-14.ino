/*
 * esp-m2-app-ms5803-14.ino
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
 *  Created on: June 04, 2024
 *      Author: Bruno Casu
 *
 *  Version 1.0 (June 04, 2024)
 */
 
#include <ESP8266WiFi.h>
#include <PolledTimeout.h>
#include <LittleFS.h>
#include <Wire.h>
#include <SimpleFTPServer.h> // Replacing <ESP8266FtpServer.h>

#define SERIAL_SPEED  115200
#define APP_VERSION "esp-m2-app-ms5803-14_v1.0"

// Defines for the data aquisition system
#define SLEEP_TIME_MS 1800000  // Interval between measurements in ms
#define GPIO_SET_ACCESS_POINT 14 // On Wemos D1 Mini - Pin number 14 (GPIO14)

// MS5803_14 defines
#define MS5803_CMD_OK 0
#define MS5803_CMD_ERROR -1
#define GPIO_2_VCC 2
// PIN 12 in the ESP-M2 is connected to the VCC of the MS5803_14BA sensor
// WARNING GPIO 12 MUST BE SET TO HIGH BEFORE SENSOR READING
#define MS5803_VCC_PIN 12
// I2C Pins
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
// I2C Addresses
const int16_t I2C_CONTROLLER_ADDR = 0x42;
const int16_t MS5803_I2C_DEVICE_ADDR = 0x77;
// MS5803 Commands
const uint8_t CMD_RESET = 0x1E;
const uint8_t CMD_CONVP = 0x40; // ADC conversion 256
const uint8_t CMD_CONVT = 0x50;
const uint8_t CMD_RDADC = 0x00;
const uint8_t CMD_RDROM = 0xA0;

// Remove when deploying in production environment
//#define DEBUG_MODE

#define CYCLE_COUNTER_RST 10000000
unsigned long cycle_counter = 0;

// WiFi Access Point configuration
IPAddress local_IP(10,10,10,1); // FTP server address
IPAddress gateway(10,10,10,1);
IPAddress subnet_mask(255,255,255,0);
const char* ssid_AP = "V32-AP-ESPM2"; // No password set

// FTP server access configuration
const char* user_FTP = "espm2";
const char* pwd_FTP = "espm2";
FtpServer ftpSrv; // Handler

// LittleFs info
FSInfo fs_info;

// Data file configuration
const char* coef_file_path = "/V32_coef.txt";
const char* coef_header_description = "C0;C1;C2;C3;C4;C5;C6;CRC;"; // Added at the creation of the Data file
const char* data_file_path = "/V32_raw_data.csv";
const char* data_csv_header_description = "Ctr;RawPressure;RawTemperature"; // Added at the creation of the Data file


/** PF definitions **/

/**
 * Error handler function
 */
void errorHandler(){
  //while(1){
  //  delay(100);
  //}
}

/**
 * Serial print the device configuration and file system info
 */
void printDeviceInfo(){
  LittleFS.info(fs_info);
  Serial.begin(SERIAL_SPEED);
  delay(500);
  Serial.print("\n\n-->SW VERSION: ");
  Serial.println(APP_VERSION);
  Serial.print(F("Cycle counter = "));
  Serial.println(cycle_counter);
  Serial.print("\n-->SLEEP_TIME_MS: ");
  Serial.println(SLEEP_TIME_MS);
  // FS info
  Serial.print("-->LittleFS Mount OK\nTotal FS Size (kB):");
  Serial.println(fs_info.totalBytes*0.001);
  Serial.print("Used (kB): ");
  Serial.println(fs_info.usedBytes*0.001);
  // FTP-AP info
  Serial.print("\n-->AP Mode - WiFi SSID (NO PASSWORD): ");
  Serial.println(ssid_AP);
  Serial.print("\n-->FTP Server IP Addr: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Plain FTP Connection (insecure)\nFTP_USR: ");
  Serial.println(user_FTP);
  Serial.print("FTP_PWD: ");
  Serial.println(pwd_FTP);

  Serial.flush();
}

/**
 * Configures the ESP8266 as WiFi Access Point, and starts the FTP server
 */
void setAccessPoint() {
  // Configure WiFi Access point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid_AP); // No password for connecting
  WiFi.softAPConfig(local_IP, gateway, subnet_mask);
  printDeviceInfo();
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
 * Create data files for each sensor
 */
void createDataFiles(){
  File new_file;
  uint16_t coef_buff[8];
  // Create Data files, if they do not exist
  if (!LittleFS.exists(coef_file_path)){ // File does not exist
    new_file = LittleFS.open(coef_file_path, "w");
    if (!new_file) {
      errorHandler(); // Failed creating file
    }
    else{
      if(ms5803_read_coefficients(coef_buff) != MS5803_CMD_OK){ // Read 8 coefficients
        errorHandler();
      }
      else{
        new_file.print(coef_header_description); // Write header
        new_file.print("\n");
        for(int k=0;k<8;k++){ // Write coefficients
          new_file.print(coef_buff[k]);
          new_file.print(";");
        }
        new_file.flush(); // Ensure writting before returning
        new_file.close();
      }
    }
  }
  if (!LittleFS.exists(data_file_path)){ // File does not exist
    new_file = LittleFS.open(data_file_path, "w");
    if (!new_file) {
      errorHandler(); // Failed creating file
    }
    else{
      new_file.print(data_csv_header_description); // Write CSV header
      new_file.flush(); // Ensure writting before returning
      new_file.close();
    }
  }
}

/**
 * Read and save sensor data on data files
 */
void dataAcquisition() {
  int p_ret, t_ret;
  unsigned long raw_pressure = 0;
  unsigned long raw_temperature = 0;
  if (LittleFS.exists(data_file_path)){ // File exists
    p_ret = ms5803_read_raw_pressure(&raw_pressure);
    t_ret = ms5803_read_raw_temperature(&raw_temperature);
    if(p_ret == MS5803_CMD_OK && t_ret == MS5803_CMD_OK){ // Readings OK - Write data
      appendData(raw_pressure, raw_temperature, data_file_path);
    }
  }
}

/**
 * Append the sensor data into the Data files
 */
void appendData(unsigned long raw_pres, unsigned long raw_temp, const char *path) {
  //int exe_time = millis();
  File data_file = LittleFS.open(path, "a");
    if (!data_file) {
      return;
    }
    else {
      data_file.print("\n");
      data_file.print(cycle_counter);
      data_file.print(";");
      data_file.print(raw_pres);
      data_file.print(";");
      data_file.print(raw_temp);
      data_file.flush(); // Ensure writting before returning
    }
    data_file.close();
#ifdef DEBUG_MODE
    Serial.print("\n-->DATA");
    Serial.print(" P: ");
    Serial.print(raw_pres);
    Serial.print(" T: ");
    Serial.print(raw_temp);
#endif // DEBUG_MODE
}

// MS5803_14 Sensor functions
/**
 * Send I2C command
 * Return 0 if Tx OK, -1 if Error
 */
int ms5803_send_cmd(uint8_t cmd){
  Wire.beginTransmission(MS5803_I2C_DEVICE_ADDR);
  Wire.write(cmd);
  delay(3);
  if (Wire.endTransmission() == 0) {
    return 0; // Command response OK
  }
  else {
    return -1; // Error in I2C transmission
  }
}

/**
 * Initialyze the MS5803 sensor: configures the GPIOs and send a Reset command
 * Return 0 if command sent is OK, and -1 if Error
 */
int ms5803_init(void){
  pinMode(MS5803_VCC_PIN, OUTPUT);
  digitalWrite(MS5803_VCC_PIN, HIGH); // Must be set to high to power the ms503 sensor
  pinMode(GPIO_2_VCC, OUTPUT);
  digitalWrite(GPIO_2_VCC, HIGH);
  //delay(100);
  Wire.setClock(100000UL);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CONTROLLER_ADDR);
  // Send a RESET command
  if (ms5803_send_cmd(CMD_RESET) != MS5803_CMD_OK){
    return -1;
  }
  else{
    return 0;
  }
}

/**
 * Read the MS5803 coefficients from the ROM memory
 * Return 0 if command sent is OK, and -1 if Error
 */
int ms5803_read_coefficients(uint16_t *coef){
  int n;
  uint8_t buff[2];
  for (int i = 0; i < 8; i++) { // Request values for 8 coefficients
    if (ms5803_send_cmd((uint8_t) (CMD_RDROM + (i << 1))) != MS5803_CMD_OK){
      return -1;
    }
    n = 0;
    buff[0]=0;
    buff[1]=0;
    Wire.requestFrom(MS5803_I2C_DEVICE_ADDR, 2); // Request 2 bytes from device
    delay(1);
    while (Wire.available() && n < 2) { // Device may send less than requested
      buff[n] = Wire.read();
      n++;
    }
    if (n != 2) {
      return -1; // Error in I2C transmission
    }
    // Copy 16 bit coefficient
    coef[i] = (uint16_t)buff[1] | ((uint16_t)buff[0] << 8);
    delay(5);
  }
  return 0;
}

/**
 * Send command to trigger the ADC conversion then request the Raw data (3 bytes)
 * Return raw data value
 */
int ms5803_read_raw_pressure(unsigned long *raw_pressure){
  int n;
  uint8_t buff[3];
  *raw_pressure = 0;
  // Send Conversion request for Pressure
  if (ms5803_send_cmd(CMD_CONVP) != MS5803_CMD_OK){
    return -1;
  }

  delay(3); // Delay for conversion

  // Send read ADC command
  if (ms5803_send_cmd(CMD_RDADC) != MS5803_CMD_OK){
    return -1;
  }

  Wire.requestFrom(MS5803_I2C_DEVICE_ADDR, 3);
  delay(1);
  n = 0;
  while (Wire.available() && n < 3) { // slave may send less than requested
    buff[n] = Wire.read(); // receive a byte as character
    *raw_pressure = (*raw_pressure << 8) + (uint8_t) (buff[n] & 0xff);
    n++;
  }
  if (n != 3) {
    return -1; // Error in I2C transmission
  }
  else{
    return 0; // Command OK
  }

}

/**
 * Send command to trigger the ADC conversion then request the Raw data (3 bytes)
 * Return raw data value
 */
int ms5803_read_raw_temperature(unsigned long *raw_temperature){
  int n;
  uint8_t buff[3];
  *raw_temperature = 0;
  // Send Conversion request for Pressure
  if (ms5803_send_cmd(CMD_CONVT) != MS5803_CMD_OK){
    return -1;
  }

  delay(3); // Delay for conversion

  // Send read ADC command
  if (ms5803_send_cmd(CMD_RDADC) != MS5803_CMD_OK){
    return -1;
  }

  Wire.requestFrom(MS5803_I2C_DEVICE_ADDR, 3);
  delay(1);
  n = 0;
  while (Wire.available() && n < 3) { // slave may send less than requested
    buff[n] = Wire.read(); // receive a byte as character
    *raw_temperature = (*raw_temperature << 8) + (uint8_t) (buff[n] & 0xff);
    n++;
  }
  if (n != 3) {
    return -1; // Error in I2C transmission
  }
  else{
    return 0; // Command OK
  }

}


void setup() {
  system_rtc_mem_read(64, &cycle_counter, 4); // Copy RTC memory value in current cycle counter
  if (cycle_counter > CYCLE_COUNTER_RST){cycle_counter = 0;} // This prevents errors in first boot if done by GPIO reset (RTC memeory gets wrong value)
  pinMode(GPIO_SET_ACCESS_POINT, INPUT_PULLUP); // Set Access point pin
  Wire.begin(); // I2C
  WiFi.mode(WIFI_OFF); // Must turn the modem off; using disconnect won't work
  WiFi.forceSleepBegin();
  // Mount file system
  if (!LittleFS.begin()) {
    errorHandler(); // LittleFS mount failed
  }
#ifdef DEBUG_MODE
  printDeviceInfo();
#endif // DEBUG_MODE  
  // Initialyze MS5803_14
  if(ms5803_init() != MS5803_CMD_OK){
    errorHandler();
  }
  // Setup data files and read the sensors
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
#ifdef DEBUG_MODE  
  Serial.print("\nEXE TIME (ms): ");
  Serial.println(millis());
#endif // DEBUG_MODE 
  // Sleep untile next cycle
  ESP.deepSleep(SLEEP_TIME_MS*1000, WAKE_NO_RFCAL); // Deep Sleep - MCU reset at wakeup - GPIO16 must be connected to RST
}


void loop() { }
