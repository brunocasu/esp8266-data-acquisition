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

#define SHT30_I2C_ADDR_PIN_HIGH 0x45
#define SHT30_I2C_ADDR_PIN_LOW 0x44
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

// SHT30 Shield
SHT3X sht30_1_handler(SHT30_I2C_ADDR_PIN_HIGH);

// SHT30 connect via cable in I2C port
SHT3X sht30_2_handler(SHT30_I2C_ADDR_PIN_LOW);

void setup() {
  Serial.println("ESP8266 Temp Monitor Setup");
  WiFi.mode(WIFI_OFF);      // you must turn the modem off; using disconnect won't work
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  Serial.begin(115200);
  Wire.begin();
}

void loop() {

  if(sht30_1_handler.get()==0){
    Serial.print("\n--------\nTemperature in Celsius : ");
    Serial.println(sht30_1_handler.cTemp);
    Serial.print("Relative Humidity : ");
    Serial.println(sht30_1_handler.humidity);
    Serial.println();
  }
  else{
    Serial.println("SHT30 get Error");
  }
  delay(100);
  setLED(BLUE_LED_ON);
  
  
  Serial.println(F("\n-----------\nEnter Timed Light Sleep"));

  // Here all the code to put con light sleep
  // the problem is that there is a bug on this
  // process
  extern os_timer_t* timer_list;
  timer_list = nullptr;  // stop (but don't disable) the 4 OS timers
  uint32_t sleep_time_in_ms = 5000; // sleep for 10s
  wifi_set_opmode(NULL_MODE);
  wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
  wifi_fpm_open();
  wifi_fpm_set_wakeup_cb(wakeupCallback);
  wifi_fpm_do_sleep(sleep_time_in_ms * 1000 );
  // callback returns, then delay extra 5 seconds...
  delay(sleep_time_in_ms + 1);
  Serial1.println("Exit light sleep mode");

}

void wakeupCallback() {  // unlike ISRs, you can do a print() from a callback function
  Serial.println(F("Woke from Light Sleep - this is the callback"));
  // setLED(BLUE_LED_ON);
}

void setLED(led_status_t cmd){
  if(cmd == BLUE_LED_ON){
    digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on (Note that LOW is the voltage level
  }
  else if(cmd == BLUE_LED_OFF){
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on (Note that LOW is the voltage level
  }
}
