#include <FS.h>
#include <LittleFS.h>
#include <time.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <PolledTimeout.h>

#define SDA_PIN 4
#define SCL_PIN 5
const int16_t I2C_MASTER = 0x42;
const int16_t I2C_SLAVE = 0x77;
const uint8_t CMD_RESET = 0x1E;
const uint8_t CMD_CONVP = 0x40;
const uint8_t CMD_CONVT = 0x50;
const uint8_t CMD_RDADC = 0x00;
const uint8_t CMD_RDROM = 0xA0;



void writeFile(const char *path, const char *message) {
  Serial.printf("Writing file: %s\n", path);

  File file = LittleFS.open(path, "w");
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  delay(2000);  // Make sure the CREATE and LASTWRITE times are different
  file.close();
}

void appendFile(const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = LittleFS.open(path, "a");
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}


void setup() {
  Serial.begin(115200);
  char buff[16];
  
  Serial.println("Mount LittleFS");
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
    return;
  }
  Serial.println("Formatting LittleFS filesystem");
  LittleFS.format();
  
  writeFile("/test_pressure_temp_data.txt", "INIT\n"); 
  appendFile("/hello.txt", "World!\n");

  // Setup I2C for reading out the pressure sensor
  Wire.setClock(100000UL);
  Wire.begin(SDA_PIN, SCL_PIN, I2C_MASTER);        // join i2c bus (address optional for master)

  // Send a RESET command
  Wire.beginTransmission(I2C_SLAVE);
  Wire.write(CMD_RESET);
  // delay(3);
  if (Wire.endTransmission() == 0) {
    Serial.print("SLAVE found at address ");
    Serial.println(I2C_SLAVE, HEX);
  }
  for (int i = 0; i < 8; i++) {
    Serial.print("Retrieving PROM coefficient ");
    Serial.print(i);
    Serial.print(" with I2C command code ");
    Serial.println(CMD_RDROM + (i << 1), HEX);
    Wire.beginTransmission(I2C_SLAVE);
    Wire.write((uint8_t) (CMD_RDROM + (i << 1)));
    Wire.endTransmission();
    // delay(5);
    Wire.requestFrom(I2C_SLAVE, 2);    // request 2 bytes from slave device #8
    // delay(5);
    int jndex = 0;
    while (Wire.available() && jndex<sizeof(buff)) { // slave may send less than requested
      buff[jndex] = Wire.read(); // receive a byte as character
      jndex++;
    }
    Serial.print(buff[0], HEX);
    Serial.println(buff[1], HEX);
    Serial.flush();
    //coeffs[i] = 256 * buff[0] + buff[1];
    //rCoeff[i] = (double) coeffs[i];
    //rtcPos = RTCMEMORYSTART;
  }
  // Copy the PROM coefficients to RTC memory
  //system_rtc_mem_write(rtcPos, rCoeff, 8 * 8);

  // Now read the pressure: send a conversion command
    Wire.beginTransmission(I2C_SLAVE);
    Wire.write(CMD_CONVP);
    result = Wire.endTransmission();
    delay(3);

    
  
}

void loop() {}
