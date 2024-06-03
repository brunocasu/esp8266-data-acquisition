#include <FS.h>
#include <LittleFS.h>
#include <time.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <PolledTimeout.h>

#define MS5803_CMD_OK 0
#define MS5803_CMD_ERROR -1
#define GPIO_2_VCC 2
// PIN 12 in the ESP-M2 is connected to the VCC of the MS5803_14BA sensor
// WARNING GPIO 12 MUST BE SET TO HIGH BEFORE SENSOR READING
#define MS5803_VCC_PIN 12
// I2C Pins
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 5
// I2C Address
const int16_t I2C_CONTROLLER_ADDR = 0x42;
const int16_t MS5803_I2C_DEVICE_ADDR = 0x77;
// MS5803 Commands
const uint8_t CMD_RESET = 0x1E;
const uint8_t CMD_CONVP = 0x40; // ADC conversion 256
const uint8_t CMD_CONVT = 0x50;
const uint8_t CMD_RDADC = 0x00;
const uint8_t CMD_RDROM = 0xA0;

// Data file configuration
const char* data_file_path = "/sensor_data.txt";

void errorHandler(){
  Serial.println("Error Handler");
  while(1){delay(100);}
}

/**
 * 
 * 
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
  Serial.begin(115200);
  uint16_t coef_buff[8];
  uint8_t serial_buff[2];
  uint8_t press_buff[3];
  int jndex;
  unsigned long raw_pressure = 0;

  pinMode(MS5803_VCC_PIN, OUTPUT);
  digitalWrite(MS5803_VCC_PIN, HIGH);
  pinMode(GPIO_2_VCC, OUTPUT);
  digitalWrite(GPIO_2_VCC, HIGH);
  //Serial.println("Mount LittleFS");
  //if (!LittleFS.begin()) {
  //  Serial.println("LittleFS mount failed");
  //  return;
  //}
  //Serial.println("Formatting LittleFS filesystem");
  //LittleFS.format();
  //
  if(ms5803_init() != MS5803_CMD_OK){
    errorHandler();
  }
  Serial.println("\nMS5803 INIT OK");
  
  if(ms5803_read_coefficients(coef_buff) != MS5803_CMD_OK){
    errorHandler();
  }
  Serial.println("COEFFICIENTS");
  for(int k=0;k<8;k++){
    Serial.print(k);
    Serial.print("-");
    Serial.println(coef_buff[k]);
  }
  
  Serial.println("RAW PRESSURE READINGS");
  if(ms5803_read_raw_pressure(&raw_pressure) != MS5803_CMD_OK){
    errorHandler();
  }
  
  Serial.println(raw_pressure);
  //writeFile("/test_pressure_temp_data.txt", "INIT\n");
  //appendFile("/hello.txt", "World!\n");

  //Setup I2C for reading out the pressure sensor
  //Wire.setClock(100000UL);
  //Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CONTROLLER_ADDR);        // join i2c bus (address optional for master)

  // Send a RESET command
  //Wire.beginTransmission(MS5803_I2C_DEVICE_ADDR);
  //Wire.write(CMD_RESET);
  //delay(3);
  //if (Wire.endTransmission() == 0) {
  //  Serial.print("SLAVE found at address ");
  //  Serial.println(MS5803_I2C_DEVICE_ADDR, HEX);
  //}
  //delay(50);
  //for (int i = 0; i < 8; i++) {
  //  Serial.print("Retrieving PROM coefficient ");
  //  Serial.print(i);
  //  Serial.print(" with I2C command code ");
  //  Serial.println(CMD_RDROM + (i << 1), HEX);
  //  Wire.beginTransmission(I2C_SLAVE);
  //  Wire.write((uint8_t) (CMD_RDROM + (i << 1)));
  //  Wire.endTransmission();
  //  delay(5);
  //  Wire.requestFrom(I2C_SLAVE, 2); // request 2 bytes from slave device #8
  //  jndex = 0;
  //  serial_buff[0]=0;
  //  serial_buff[1]=0;
  //  while (Wire.available() && jndex<2) { // slave may send less than requested
  //    serial_buff[jndex] = Wire.read(); // receive a byte as character
  //    jndex++;
  //  }
  //  Serial.print(serial_buff[0], HEX);
  //  Serial.println(serial_buff[1], HEX);
  //  coef_buff[i] = (uint16_t)serial_buff[1] | ((uint16_t)serial_buff[0] << 8);
  //  Serial.print("INT VALUE: ");
  //  Serial.println(coef_buff[i]);
  //  delay(50);
  //}
  // Copy the PROM coefficients to RTC memory
  //system_rtc_mem_write(rtcPos, rCoeff, 8 * 8);

// Now read the pressure: send a conversion command
  //Wire.beginTransmission(I2C_SLAVE);
  //Wire.write(CMD_CONVP);
  //int result = Wire.endTransmission();
  //delay(3);
  //// Send a command to read the ADC
  //Wire.beginTransmission(I2C_SLAVE);
  //Wire.write(CMD_RDADC);
  //result = Wire.endTransmission();
  //// Request 3 bytes of conversion results
  //Wire.beginTransmission(I2C_SLAVE);
  //Wire.requestFrom(I2C_SLAVE, 3);
  //result = Wire.endTransmission();
  //// Actually read the 3 bytes of data back and rearrange them
  //result = Wire.available();
  //Serial.print("Pressure read request gave ");
  //Serial.println(result);
  //unsigned long iPressure = 0;
  //jndex = 0;
  //while (Wire.available()&& jndex<3) { // slave may send less than requested
  //  press_buff[jndex] = Wire.read(); // receive a byte as character
  //  iPressure = (iPressure << 8) + (uint8_t) (press_buff[jndex] & 0xff);
  //  Serial.println(press_buff[jndex]);
  //  jndex++;
  //}
   
  
}

void loop() {
  unsigned long raw_pressure = 0;
  unsigned long raw_temperature = 0;
  if(ms5803_read_raw_pressure(&raw_pressure) != MS5803_CMD_OK){
    errorHandler();
  }
  if(ms5803_read_raw_temperature(&raw_temperature) != MS5803_CMD_OK){
    errorHandler();
  }
  
  Serial.print(raw_pressure);
  Serial.print(";");
  Serial.println(raw_temperature);
  delay(1000);  
}
