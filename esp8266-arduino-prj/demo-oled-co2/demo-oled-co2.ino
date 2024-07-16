/*
 * demo-oled-co2.ino
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
 *  Created on: July 16, 2024
 *      Author: Bruno Casu
 */

#include "Arduino.h"
#include "cozir.h"
#include "SoftwareSerial.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <sht30.h>
#include <LOLIN_HP303B.h>

// SCL GPIO5
// SDA GPIO4
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);

#define SHT30_I2C_ADDR_PIN_HIGH 0x45  // Jumper NOT connected
#define SHT30_I2C_ADDR_PIN_LOW 0x44  // Jumper connected
#define HP303B_I2C_ADDR_PIN_HIGH 0x77  // Jumper NOT connected
#define HP303B_OVERSAMPLING_RATE  2  // From 1 to 7 (7 is  the highest oversampling rate)

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2


#define LOGO16_GLCD_HEIGHT 16
#define LOGO16_GLCD_WIDTH  16
static const unsigned char PROGMEM logo16_glcd_bmp[] =
{ B00000000, B11000000,
  B00000001, B11000000,
  B00000001, B11000000,
  B00000011, B11100000,
  B11110011, B11100000,
  B11111110, B11111000,
  B01111110, B11111111,
  B00110011, B10011111,
  B00011111, B11111100,
  B00001101, B01110000,
  B00011011, B10100000,
  B00111111, B11100000,
  B00111111, B11110000,
  B01111100, B11110000,
  B01110000, B01110000,
  B00000000, B00110000 };

#if (SSD1306_LCDHEIGHT != 48)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// Temperature and humidity sensor
SHT30 sht30_1_handler(SHT30_I2C_ADDR_PIN_HIGH); // Addr 0x45
SHT30 sht30_2_handler(SHT30_I2C_ADDR_PIN_LOW); // Addr 0x44
// Temperature and pressure sensor
LOLIN_HP303B hp303b_handler;

SoftwareSerial sws(13, 12);  // RX, TX, optional inverse logic

COZIR czr(&sws);


void setup()
{
  sws.begin(9600);
  czr.init();

  Serial.begin(115200);
  Serial.print("COZIR_LIB_VERSION: ");
  Serial.println(COZIR_LIB_VERSION);
  Serial.println();

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  // init done
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();

  hp303b_handler.begin();

}


void loop()
{
  static int loop_ctr = 0;
  
  char temp_arr[10];
  char *temp_header = " C";
  int temp_header_size = 2;
  
  char hum_arr[10];
  char *hum_header = " %RH";
  int hum_header_size = 4;
  
  char pres_arr[12];
  char *pres_header = " Pa";
  int pres_header_size = 3;
  
  char co2_arr[10];
  char *co2_header = " %CO2";
  int co2_header_size = 5;
  
  static float temp;
  static float hum;
  static int pres;
  static float co2;

  float read_co2 = (float)readCozirStream();
  Serial.println(read_co2);
  if(read_co2>0){co2 = read_co2/100;}
  Serial.print(co2);
  
  delay(500);
  if(sht30_2_handler.read_single_shot() == SHT30_READ_OK){
    temp = sht30_2_handler.cTemp;
    hum = sht30_2_handler.humidity;
  }
  delay(500);
  int32_t read_pres=0;
  hp303b_handler.measurePressureOnce(read_pres, 7);
  if(read_pres>0){
    pres=read_pres;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);

  int size_arr = float_to_char_array(co2, co2_arr);
  for (int i=0;i<size_arr;i++){
    testdrawchar(co2_arr[i]);
  }
  for (int i=0;i<co2_header_size;i++){
    testdrawchar(co2_header[i]);
  }
  display.println();

  size_arr = float_to_char_array(temp, temp_arr);
  for (int i=0;i<size_arr;i++){
    testdrawchar(temp_arr[i]);
  }
  for (int i=0;i<temp_header_size;i++){
    testdrawchar(temp_header[i]);
  }
  display.println();

  size_arr = float_to_char_array(hum, hum_arr);
  for (int i=0;i<size_arr;i++){
    testdrawchar(hum_arr[i]);
  }
  for (int i=0;i<hum_header_size;i++){
    testdrawchar(hum_header[i]);
  }
  display.println();

  size_arr = int_to_char_array(pres, pres_arr);
  for (int i=0;i<size_arr;i++){
    testdrawchar(pres_arr[i]);
  }
  for (int i=0;i<pres_header_size;i++){
    testdrawchar(pres_header[i]);
  }
  display.println();
  
  
  //display.clearDisplay();
  delay(2000);
  loop_ctr++;
}

int parseSensorValue(String input) {
  // Find the position of 'Z' in the input string
  int zPos = input.indexOf('Z');
  
  // If 'Z' is found
  if (zPos != -1) {
    // Extract the substring starting from 'Z' position
    String numberString = input.substring(zPos + 2); // Skip 'Z ' to get the number part
    
    // Trim any leading or trailing whitespace
    numberString.trim();
    
    // Convert the string to an integer
    int sensorValue = numberString.toInt();
    
    // Return the parsed integer value
    return sensorValue;
  }
  
  // If 'Z' is not found or parsing fails, return a default value or handle the error appropriately
  return -1;
}

int readCozirStream(void){
  czr.setOperatingMode(CZR_STREAMING);
  String result = "";
  char currentChar;
  
  // Wait for 'Z' character
  while (sws.available()) {
    currentChar = sws.read();
    if (currentChar == 'Z') {
      result += currentChar;
      break;
    }
  }
  
  // If 'Z' character detected, read the next 6 characters
  if (currentChar == 'Z') {
    for (int i = 0; i < 6; i++) {
      while (!sws.available()); // Wait until data available
      currentChar = sws.read();
      result += currentChar;
    }
  }
  
  Serial.println(result);
  int sensorValue = parseSensorValue(result);
  // Print the parsed sensor value
  if(sensorValue > 0){
    return sensorValue;
  }
  else {
    return 0;
  }
  
}


int float_to_char_array(float value, char *output) {
    int chars_written;
    // Convert the float to a string with maximum 2 decimal points
    chars_written = snprintf(output, 10, "%.2f", value);
      
    // Check for buffer overflow
    if (chars_written >= 10) {
        fprintf(stderr, "Error: Buffer overflow occurred.\n");
        return -1; // Return error code
    }
    return chars_written; // Return the number of characters written
}

int int_to_char_array(int value, char *output) {
    int chars_written;
    // Convert the float to a string with maximum 2 decimal points
    chars_written = snprintf(output, 10, "%d", value);
      
    // Check for buffer overflow
    if (chars_written >= 10) {
        fprintf(stderr, "Error: Buffer overflow occurred.\n");
        return -1; // Return error code
    }
    return chars_written; // Return the number of characters written
}

//float calculate_ppmv(float T_t, float P_t, float RH_t){
//  float ews_t; // 
  
//}


void testdrawchar(char input_char) {
  
  display.write(input_char);

  display.display();
  delay(1);
}

void testdrawline() {
  for (int16_t i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, WHITE);
    display.display();
    delay(1);
  }
  for (int16_t i=0; i<display.height(); i+=4) {
    display.drawLine(0, 0, display.width()-1, i, WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();
  for (int16_t i=0; i<display.width(); i+=4) {
    display.drawLine(0, display.height()-1, i, 0, WHITE);
    display.display();
    delay(1);
  }
  for (int16_t i=display.height()-1; i>=0; i-=4) {
    display.drawLine(0, display.height()-1, display.width()-1, i, WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();
  for (int16_t i=display.width()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, i, 0, WHITE);
    display.display();
    delay(1);
  }
  for (int16_t i=display.height()-1; i>=0; i-=4) {
    display.drawLine(display.width()-1, display.height()-1, 0, i, WHITE);
    display.display();
    delay(1);
  }
  delay(250);

  display.clearDisplay();
  for (int16_t i=0; i<display.height(); i+=4) {
    display.drawLine(display.width()-1, 0, 0, i, WHITE);
    display.display();
    delay(1);
  }
  for (int16_t i=0; i<display.width(); i+=4) {
    display.drawLine(display.width()-1, 0, i, display.height()-1, WHITE);
    display.display();
    delay(1);
  }
  delay(250);
}


// -- END OF FILE --
