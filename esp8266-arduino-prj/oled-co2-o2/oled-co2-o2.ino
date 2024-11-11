/*
 * oled-co2-o2.ino
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
 *  Created on: November 06, 2024
 *      Author: Bruno Casu
 *      
 *  Version 1.0 (November 06, 2024)
 */

#include "Arduino.h"
#include "cozir.h"
#include "SoftwareSerial.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <sht30.h> // custom sht30 library
//#include <LOLIN_HP303B.h>


#define LOX_O2_MAX_READ_ATTEMPTS  5
#define SHT30_I2C_ADDR_PIN_HIGH 0x45  // Jumper NOT connected
//#define SHT30_I2C_ADDR_PIN_LOW 0x44  // Jumper connected
//#define HP303B_I2C_ADDR_PIN_HIGH 0x77  // Jumper NOT connected
//#define HP303B_OVERSAMPLING_RATE  2  // From 1 to 7 (7 is  the highest oversampling rate)

// OLED Defines
// SCL GPIO5
// SDA GPIO4
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);
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

// SHT30: Temperature and humidity sensor
SHT30 sht30_1_handler(SHT30_I2C_ADDR_PIN_HIGH); // Addr 0x45

// Luminox LOX-O2-F: 0-25% O2 sensor + Temperature and pressure
SoftwareSerial swsLox(14, 15); // GPIO 15 (TX) goes into LOX-02 Pin 4. GPIO 14 (RX) goes into LOX-02 Pin 3.

// SPRINTIR-WF-100: 0-100% CO2 sensor
SoftwareSerial swsCozir(13, 12);  // RX, TX, optional inverse logic
COZIR czr(&swsCozir);


void setup()
{
  swsLox.begin(9600); // Start serial for O2 sensor
  swsCozir.begin(9600); // Start serial for CO2 sensor
  czr.init();

  Serial.begin(115200); // USB serial (debug only)

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);
  // Clear the buffer.
  display.clearDisplay();
  displayHeader();
}


void loop()
{
  String lox_data = readLoxStream(LOX_O2_MAX_READ_ATTEMPTS); 
  String czr_data = readCozirStream(LOX_O2_MAX_READ_ATTEMPTS);
  printData(lox_data, czr_data);
  if (lox_data != "error" && czr_data != "error"){
    writeData(lox_data, czr_data); // Refresh the data on the display
  }
  delay(1000);
}


/**
 * 
 * 
 */
void printData(String lox_input, String czr_input){
  static int minutes;
  int seconds = millis() % 60000;
  minutes = millis()/60000;
  Serial.print("\nTime (m:s): ");
  Serial.print(minutes);
  Serial.print(":");
  Serial.print(seconds/1000);
  Serial.print("  LOX-O2 DATA: ");
  Serial.print(lox_input);
  Serial.print("  Cozir CO2 DATA: ");
  Serial.println(czr_input);
}

/**
 *
 *
 */
int writeData(String lox_input, String czr_input) {
  // input String should in the format (example): T +23.4 P 1028 % 019.97
  // Find positions of 'T ', 'P ', and '% ' in the string
  int tPos = lox_input.indexOf("T ");
  int pPos = lox_input.indexOf(" P ");
  int percentPos = lox_input.indexOf(" % ");

  // Extract and parse temperature value after "T "
  String tempStr = lox_input.substring(tPos + 3, tPos + 7); // Extract Temperature value

  // Extract and parse pressure value after "P "
  String pressStr = lox_input.substring(pPos + 3, pPos + 7); // Extract Pressure value

  // Extract and parse percentage value after "% "
  String percentStr = lox_input.substring(percentPos + 4, percentPos + 8); // Extract O2% value

  diplayRefreshData(tempStr, pressStr, percentStr, czr_input);
  return 1;
}

/**
 * 
 * 
 */
void displayHeader (void){
  display.setTextColor(WHITE, BLACK);
  display.setTextSize(1);
  display.setCursor(48,0); // setCursor reference the pixels (64x48)
  display.print("%C");
  display.setCursor(48,16);
  display.print("%O");
  display.setCursor(32,32); // setCursor reference the pixels (64x48)
  display.print("C");
  display.setCursor(32,40);
  display.print("mbar");
  display.display();
}


/**
 *
 */
void diplayRefreshData(String temp, String press, String o2_percentage, String co2_percentage){
  display.setTextColor(WHITE, BLACK);
  
  display.setTextSize(2);
  display.setCursor(0,0);
  display.print(co2_percentage);
  display.setCursor(0,16);
  display.print(o2_percentage);

  display.setTextSize(1);
  display.setCursor(0,32);
  display.println(temp);
  display.print(press);
  display.display();
}


/**
 * Set the LOX-O2 sensor to Streaming mode and reads the O2% data from the stream
 * Returns the data string
 * 
 */
String readLoxStream(int max_attempts){
  String lox2_stream_str = "";
  char current_char;
  int validation_result=0;
  int r=0;
  // Wait for 'T' character
  while(r<max_attempts && validation_result == 0){ // If error in data string, attemp again to read sensor
    while (swsLox.available()) {
      current_char = swsLox.read();
      // LOX-O2-F streams format (example): O 0205.4 T +23.4 P 1028 % 019.97 e 0000
      if (current_char == 'T') {
        lox2_stream_str += current_char;
        for (int i = 0; i < 22; i++) { // If 'T' character detected, read next 21 chars (extract only relevant part of data stream)
          while (!swsLox.available()); // Wait until data available
          current_char = swsLox.read();
          lox2_stream_str += current_char;
        }
        break; // break from serial reading loop after finding the 'T' char and reading the next 21 chars
      }
    }
    validation_result = validateLoxStream(lox2_stream_str); // Check if copied string is valid (no errors)
    if (validation_result == 1){ // Validation returns 1 if lox2_stream_str has no errors
      return lox2_stream_str;
    }
    else{
      Serial.print("LOX-O2 DATA STRING VALIDATION ERROR: ");
      Serial.println(lox2_stream_str);
    }
    r++;
    lox2_stream_str = "";
    delay(100);
  }
  return "error";
}

/**
 *
 *
 */
int validateLoxStream(String input){
  // input String should in the format (example): T +23.4 P 1028 % 019.97
  // Find positions of 'T ', 'P ', and '% ' in the string
  int tPos = input.indexOf("T ");
  int pPos = input.indexOf(" P ");
  int percentPos = input.indexOf(" % ");

  // Ensure all sections are found and in correct order
  if (tPos == -1 || pPos == -1 || percentPos == -1 || tPos > pPos || pPos > percentPos) {
    return 0; // String contains ERROR
  }
  
  // Validate percentage section: must be in the format "###.##" after "% "
  String percentStr = input.substring(percentPos + 3); // Get substring starting after "% "
  // Check if percentStr is exactly 6 characters and in the expected format
  if (percentStr.length() != 6 || 
      !isDigit(percentStr[0]) || !isDigit(percentStr[1]) || !isDigit(percentStr[2]) || 
      percentStr[3] != '.' || 
      !isDigit(percentStr[4]) || !isDigit(percentStr[5])) {
      return 0; // ERROR: Invalid percentage format
  }

  return 1; // String OK

}


/**
 * Set the Cozir sensor to Streaming mode and reads the first full string sent
 * Return the CO2 measurement in ppm/100
 *
 */
String readCozirStream(int max_attempts){
  String result = "";
  char currentChar;
  String co2_fixed_percentage;
  int sensorValue = 0;
  //swsCozir.begin(9600);
  //czr.init();
  czr.setOperatingMode(CZR_STREAMING);

  // Wait for 'Z' character
  while (swsCozir.available()) {
    currentChar = swsCozir.read();
    if (currentChar == 'Z') {
      result += currentChar;
      break;
    }
  }

  // If 'Z' character detected, read the next 6 characters
  if (currentChar == 'Z') {
    for (int i = 0; i < 6; i++) {
      while (!swsCozir.available()); // Wait until data available
      currentChar = swsCozir.read();
      result += currentChar;
    }
  }
  //swsCozir.end();
  //Serial.print("\n-->Cozir STRING READ: ");
  //Serial.println(result);
  sensorValue = parseCozirStream(result); // sensorValue in PPM
  //Serial.print("\n-->Cozir PPM: ");
  //Serial.println(sensorValue);
  if(sensorValue > 0){
    co2_fixed_percentage = formatCozirReading(sensorValue);
    return co2_fixed_percentage;
  }
  else {
    return "error";
  }
}


/**
 * This function parses the inut string to retrieve the CO2 measurement
 * The Cozir sensor on Streaming mode sends characters as the example: "Z 00026"
 * This function returns the numerical value (int) of the parsed input string
 * THe value, when multipled by 100, corrsponds to the CO2 measurement in PPM
 */
int parseCozirStream(String input) {
  // Find the position of 'Z' in the input string
  int zPos = input.indexOf('Z');

  // Check if the string is exactly 7 characters long
  if (input.length() != 7) {
      return -1; // ERROR: Incorrect length
  }

  // Check if the first character is 'Z' and the second is a space
  if (input[0] != 'Z' || input[1] != ' ') {
      return -1; // ERROR: Missing 'Z ' at the beginning
  }

  // Check if the remaining 5 characters are all digits
  for (int i = 2; i < 7; i++) {
      if (!isDigit(input[i])) {
          return -1; // ERROR: Non-digit character found
      }
  }
  
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


/**
 * 
 * 
 */
String formatCozirReading (int val_ppm){
  float percentage = val_ppm+900;
  String result;
  percentage = percentage/100; // Convert PPM to Percentage
  // Check if value is 10.0 or higher, use one decimal place
  if (percentage >= 10.0) {
      result = String(percentage, 1);  // Format with 1 decimal place
  } 
  // Otherwise, use two decimal places
  else {
      result = String(percentage, 2);  // Format with 2 decimal places
  }

  // Ensure the result is exactly 4 characters by trimming or padding if needed
  if (result.length() > 4) {
      result = result.substring(0, 4);  // Trim to 4 characters if too long
  } else {
      while (result.length() < 4) {
          result += "0";  // Pad with trailing zeroes if less than 4 characters
      }
  }
  return result;
}
