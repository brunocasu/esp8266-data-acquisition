//
//    FILE: Cozir_SWSerial_CO2_only.ino
//  AUTHOR: Rob Tillaart
// PURPOSE: demo of Cozir lib (>= 0.1.06)
//     URL: https://github.com/RobTillaart/Cozir
//
// NOTE: software serial is less reliable than hardware serial


#include "Arduino.h"
#include "cozir.h"
#include "SoftwareSerial.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

  // draw many lines
  testdrawline();
  display.display();
  delay(1000);
  display.clearDisplay();
  
  czr.setOperatingMode(CZR_STREAMING);
  uint32_t ppm = czr.getPPMFactor();
  Serial.print("  /  PPMFactor =\t");
  Serial.println(ppm);
  int k=0;
  // for echo in continuous mode.
  while (k<100){
      if (sws.available()){
        //Serial.print("SWS DATA: ");
        Serial.write(sws.read());
        k++;
    } 
  }
  delay(1000);
  // set to polling explicitly.
  czr.setOperatingMode(CZR_POLLING);
  delay(1000);
}


void loop()
{
  static int loop_ctr = 0;
  char co2_arr[10];
  char *co2_header = " %CO2";
  int co2_header_size = 5;
  float co2 = czr.CO2();
  delay(1000);
  co2 /= 100;  // most of time PPM = one.
  int size_arr = float_to_char_array(co2, co2_arr);

   if (size_arr != -1) {
        Serial.print("Sensor Reading: ");
        Serial.print(co2_arr);
        Serial.println("%CO2");
    }
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  for (int i=0;i<size_arr;i++){
    testdrawchar(co2_arr[i]);
  }
  for (int i=0;i<co2_header_size;i++){
    testdrawchar(co2_header[i]);
  }
  //display.println();
  display.clearDisplay();
  delay(1000);
  loop_ctr++;
}

int float_to_char_array(float value, char *output) {
    // Convert the float to a string with maximum 2 decimal points
    int chars_written = snprintf(output, 10, "%.2f", value);
    
    // Check for buffer overflow
    if (chars_written >= 10) {
        fprintf(stderr, "Error: Buffer overflow occurred.\n");
        return -1; // Return error code
    }

    return chars_written; // Return the number of characters written
}

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
