/*
 * gas-analyzer-control.ino
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
 *  Created on: July 17, 2025
 *      Author: Bruno Casu
 *
 *  Version 0.1 (July 2025)
 */

#include "Arduino.h"
#include <SPI.h>
#include <WiFiNINA.h>
#include <cozir.h>
#include "arduino_secrets.h"
#include <Wire.h>
#include "wiring_private.h"


/* DEFINES */
#define SW_VERSION "gas-analyzer-control-v0.1"
#define SW_DATE "July 2025"

#define GPIO_SERIAL_2_TX        0  // Function D0
#define GPIO_SERIAL_2_RX        1  // Function D1
#define GPIO_PWM_MOTOR_1        2  // Function D2
#define GPIO_SENSOR_PWR_CTRL    3  // Function D3
//#define NOT_IN_USE              4  // Function D4
#define GPIO_SENSOR_CALIBRATION 5  // Function D5
#define GPIO_LED_GREEN          6  // Function D6
#define GPIO_LED_RED            7  // Function D7
#define GPIO_RELAY_1            8  // Function D8
//#define NOT_IN_USE              9  // Function D9
//#define NOT_IN_USE              10 // Function D10
//#define RESERVED_I2C_SDA        11 // Function D11
//#define RESERVED_I2C_SCL        12 // Function D12
//#define RESERVED_SERIAL_1_RX    13 // Function D13
//#define RESERVED_SERIAL_1_TX    14 // Function D14

#define EVENT_CODE_CRITICAL_ERROR 1
#define EVENT_CODE_STARTUP_ERROR  2
// #define EVENT_CODE_CALIBRATION    3
#define EVENT_CODE_SETUP          4
#define EVENT_CODE_RUNNING        5
#define EVENT_CODE_AP_CONN        6
// #define EVENT_CODE_LAN_CONN       7

#define RELAY_CTRL_CRIT_SHUT_OFF    0
#define RELAY_CTRL_NORMAL_SHUT_OFF  1
#define RELAY_CTRL_PWR_ON           2

#define PWM_CTRL_POWER_0            0   // motor off
#define PWM_CTRL_POWER_100          255 // 100% power
#define PWM_CTRL_POWER_80           204 // 80% power
#define PWM_CTRL_POWER_50           128 // 50% power
#define PWM_CTRL_POWER_20           51  // 20% power

#define COZIR_MAX_READINGS_ATTEMPTS 5
#define COZIR_SERIAL_SPEED          38400 // SprintIR-R-100
//#define COZIR_SERIAL_SPEED        9600  // SprintIR-WF-100

#define LOX_MAX_READINGS_ATTEMPTS   3
#define LOX_SERIAL_SPEED            9600

//#define TIMER_CLK_DIV_100_MS_INTERVAL 4688
//#define TIMER_CLK_DIV_800_MS_INTERVAL 37500
#define TIMER_INTERVAL_100_MS       100
#define TIMER_INTERVAL_800_MS       800
#define TIMER_INTERVAL_1000_MS      1000
#define TIMER_INTERVAL_10000_MS     10000   // 10 Seconds
#define TIMER_INTERVAL_1800000_MS   1800000 // 30 Minutes


#define FSM_RESET     0
#define FSM_SETUP     1
#define FSM_AP_SERVER 2

#define DATA_ARRAY_SIZE 48

#define DEBUG_MODE

/* PV */
int global_state = FSM_RESET;
int global_daq_interval = TIMER_INTERVAL_10000_MS;
int global_motor_default_power = PWM_CTRL_POWER_80;

char ssid[] = AP_SSID;  // Network SSID
char pass[] = AP_PASS;  // Network password (use for WPA, or use as key for WEP)
int keyIndex = 0;       // Network key index number (needed only for WEP)

COZIR czr(&Serial1);  // Cozir sensor handler
// Create new UART instance for LOX sensor
Uart SerialN2 (&sercom3, GPIO_SERIAL_2_RX, GPIO_SERIAL_2_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);

int statusWiFi = WL_IDLE_STATUS;
WiFiServer server(80);
WiFiClient client;

float global_co2 = 0; // CO2 last reading (in %)
float global_o2 = 0;  // O2 last reading (in %)
float global_o2_pp = 0; // O2 partial pressure (in mbar)
float global_gas_temp = 0; // Gas temperature (in deg. Celsius)
float global_gas_press = 0; // Gas pressure (in mbar)
float global_motor_1_pwr = 0; // Current power for motor 1 (in %)
float co2_data[DATA_ARRAY_SIZE];
float o2_data[DATA_ARRAY_SIZE];
float gas_temp_data[DATA_ARRAY_SIZE];
float gas_press_data[DATA_ARRAY_SIZE];

/* PFD */
// Cozir CO2 sensor
int parseCozirStream(String input);
int readCozirStream(void);
int initCozirDevice(void);
int calibrateCozirDevice(void);
// Lox O2 sensor
int parseLoxStream(String input, float *o2_pp, float *o2_percent, float *temp, float *press);
int readLoxStream(float *o2_pp, float *o2_percent, float *temp, float *press);
int initLoxDevice(void);
// Power control
void updatePowerMotorCtrl(int pwr_config, int motor_ctrl_pin);
void pwmMotorCtrl(int pwr, int motor_ctrl_pin);
void relayMotorCtrl(int motor_ctrl_pin, int relay_pin, int op);
void sensorCtrlReset(void);
// HTML handlers
void htmlGetRequestHandler(String req);
void handleRoot(WiFiClient &client);
void handleSensorValues(WiFiClient &client);
void handleSensorData(WiFiClient &client);
void handleMotorCtrl(WiFiClient &client);
void handleNotFound(WiFiClient &client);
// WiFi
void printWiFiStatus(void);

/* PF */
/**
 * Function for updating the status of a motor (what is the current power setup)
 * pwr_config: PWM power configuration
 * motor_ctrl_pin: GPIO of the motor
 */
void updatePowerMotorCtrl(int pwr_config, int motor_ctrl_pin){
  if(motor_ctrl_pin == GPIO_PWM_MOTOR_1){
    if(pwr_config <= 0){
      global_motor_1_pwr = 0; // Motor is off
    }
    else{
      global_motor_1_pwr = 100*((float)pwr_config/255); // Calculate power given to the motor
    }
  }
}


/**
 * Control function for PWM for connected motors
 * pwr: from 1 to 255 (255 is 100% power)
 * if pwr = 0, Shut down motor
 * motor_ctrl_pin: GPIO to be controlled
 */
void pwmMotorCtrl(int pwr, int motor_ctrl_pin){
  if(pwr <= 0){
    analogWrite(motor_ctrl_pin, 0); // Power off motor
  }
  else{
    analogWrite(motor_ctrl_pin, pwr); // Setup PWM
  }
  updatePowerMotorCtrl(pwr, motor_ctrl_pin); // Update the global with the new power
}


/**
 * Control function for activating or deactivating Relays
 * motor_ctrl_pin: PWM pin for the target motor
 * relay_pin: GPIO connected to the relay
 * op: operation code
 */
void relayMotorCtrl(int motor_ctrl_pin, int relay_pin, int op){
  switch (op) {
    case RELAY_CTRL_CRIT_SHUT_OFF:
      // WARNING! In CRITICAL MOTOR SHUT DOWN the MCU might be restarted
      analogWrite(motor_ctrl_pin, 0); // Power off motor via MOSFET
      digitalWrite(relay_pin, LOW); // Cut off VCC for the motor via RELAY
      break;
    case  RELAY_CTRL_NORMAL_SHUT_OFF: // Graceful motor shut down
      analogWrite(motor_ctrl_pin, 0); // First Power off motor via MOSFET
      delay(500); // Delay added to prevent voltage drop when switching the motor
      digitalWrite(relay_pin, LOW); // Cut off VCC for the motor via RELAY
      break;
    case RELAY_CTRL_PWR_ON: // Motor start
      if (digitalRead(relay_pin) == LOW){
        analogWrite(motor_ctrl_pin, 0); // Power off motor via MOSFET
        delay(100); // Delay added to prevent voltage drop when switching the motor
        digitalWrite(relay_pin, HIGH); // Connect VCC to the motor via RELAY
        delay(100);
      }
      break;
    default:
      break;
  }
}


/**
 * Reset connected sensors
 */
void sensorCtrlReset(void){
  digitalWrite(GPIO_SENSOR_PWR_CTRL, LOW); // Power off the sensors
  delay(50);
  digitalWrite(GPIO_SENSOR_PWR_CTRL, HIGH); // Power on the sensors
  delay(500);
}


/**
 * This function parses the input string to retrieve the CO2 measurement
 * The Cozir sensor on Streaming mode sends characters as the example: "Z 00026"
 * This function returns the numerical value (int) of the parsed input string
 * sensor_value x 100 = CO2 concentration in ppm
 */
int parseCozirStream(String input) {
  // Find the position of 'Z' in the input string
  int zPos = input.indexOf('Z');
  int sensor_value = 0;
  if (zPos != -1) {
    // Extract the substring starting from 'Z' position
    String numberString = input.substring(zPos + 2);
    numberString.trim();
    sensor_value = numberString.toInt();
    return sensor_value;
  }
  // If 'Z' is not found or parsing fails
  return -1;
}


/**
 * From the Cozir Streaming mode: read the first full string sent
 * Return the CO2 measurement in ppm/100
 */
int readCozirStream(void){
  int k = 0;
  int iteration_limit = 0; //
  String result = "";
  char currentChar;
  int sensorValue = 0;
  // Wait for 'Z' character
  // while (Serial1.available()) {
  while (iteration_limit < 2000) {
    iteration_limit++;
    currentChar = Serial1.read();
    // Serial.print(currentChar);
    if (currentChar == 'Z') { // exit loop when Z char is detected
      result += currentChar;
      break;
    }
  }

  while(k < COZIR_MAX_READINGS_ATTEMPTS && sensorValue <= 0){
    // If 'Z' character detected, read the next 6 characters
    if (currentChar == 'Z') {
      for (int i = 0; i < 6; i++) {
        while (!Serial1.available()); // Wait until data available
        // Serial.print(currentChar); // Debug
        currentChar = Serial1.read();
        result += currentChar;
      }
    }
    k++;
#ifdef DEBUG_MODE
    Serial.print("\n-->Cozir string read: ");
    Serial.print(result);
#endif
    sensorValue = parseCozirStream(result);
  }
#ifdef DEBUG_MODE
  Serial.print("\n-->Cozir parsed sensor value: ");
  Serial.print(sensorValue);
#endif
  if(sensorValue > 0){
    return sensorValue;
  }
  else {
    return 0;
  }
}


/**
 * Calibrate Cozir sensor
 * WARNING The sensor must be calibrated ONLY WHEN SUBMITTED TO ATMOSPHERIC AIR
 * CO2 readings after calibration should be around 400 ppm in atmospheric air
 * Must be used with care
 */
int calibrateCozirDevice(void){
  czr.setOperatingMode(CZR_POLLING);
  delay(500);
  Serial.println(czr.calibrateFreshAir());
  delay(500);
  return initCozirDevice();
}


/**
 * Initialyze Cozir sensor
 * User implementation: Setup Serial interface for the sensor
 *
 */
int initCozirDevice(void){
  // In Arduino MKR WiFi 1010, Serial1 uses pins D14(Controller TX) and D13(Controller RX)
  Serial1.begin(COZIR_SERIAL_SPEED);
  czr.init();
  // Set sensor to STREAMING MODE (continuous write to serial)
  czr.setOperatingMode(CZR_STREAMING);
  delay(50);
  // Test reading the sensor
  if (readCozirStream() > 0){
    return 1;
  }
  else {
    return 0;
  }
}


/**
 * This function parses the input string and retrieves the measurements
 * The LOX O2 sensor on Streaming mode sends strings as:
 * "O xxxx.x T yxx.x P xxxx % xxx.xx e xxxx"
 * O xxxx.x is O2 partial pressure in mbar
 * T yxx.x is temperature in Celsius
 * P xxxx is pressure in mbar
 * % xxx.xx is the Oxygen concentration
 * e xxxx is the sensor status
 */
int parseLoxStream(String input, float *o2_pp, float *o2_percent, float *temp, float *press){
  int found = 0;
  int oIndex = input.indexOf("O ");
  int tIndex = input.indexOf("T ");
  int pIndex = input.indexOf("P ");
  int percIndex = input.indexOf("% ");

  if (oIndex != -1 && oIndex + 7 < input.length()) {
    *o2_pp = input.substring(oIndex + 2, oIndex + 8).toFloat();
    found++;
  }
  if (tIndex != -1 && tIndex + 6 < input.length()) {
    *temp = input.substring(tIndex + 2, tIndex + 7).toFloat();
    found++;
  }
  if (pIndex != -1 && pIndex + 5 < input.length()) {
    *press = input.substring(pIndex + 2, pIndex + 6).toFloat();
    found++;
  }
  if (percIndex != -1 && percIndex + 7 < input.length()) {
    *o2_percent = input.substring(percIndex + 2, percIndex + 8).toFloat();
    found++;
  }
  if (found == 4){ // All values are parsed
    return 1;
  }
  else {
    return 0; // Failed to parse one or more values
  }
}


/**
 * Read the characters sent in Streaming mode from the LOX O2 sensor
 * LOX-O2-F streams format (example): O 0205.4 T +23.4 P 1028 % 019.97 e 0000
 * Returns 1 if reading is Ok - 0 if error in reading stream
 */
int readLoxStream(float *o2_pp, float *o2_percent, float *temp, float *press){
  String lox_stream_str = "";
  char current_char;
  int validation_result=0;
  int r=0;
  // Wait for 'T' character
  // while(r<3 && validation_result == 0){
  while(r < LOX_MAX_READINGS_ATTEMPTS){
    while (SerialN2.available()) {
      current_char = SerialN2.read();
      if (current_char == 'O') {
        lox_stream_str += current_char;
        for (int i = 0; i < 38; i++) {
          while (!SerialN2.available()); // Wait until data available
          current_char = SerialN2.read();
          lox_stream_str += current_char;
        }
        break; // Exit loop after reading stream 38 chars
      }
    }
#ifdef DEBUG_MODE
    Serial.print("\n-->LOX-O2 Stream: ");
    Serial.print(lox_stream_str);
#endif
    if (parseLoxStream(lox_stream_str, o2_pp, o2_percent, temp, press)){
      if (*o2_percent > 0){
#ifdef DEBUG_MODE
        Serial.print("\n-->Lox Parsed values - O2% reading: ");
        Serial.print(*o2_percent);
        Serial.print(" Temp(C) reading: ");
        Serial.print(*temp);
#endif
        // Exit loop when a string is parsed correctly and O2 percentage has a meaningfull value
        break;
      }
    }
    // Attempt a new reading
    r++;
    lox_stream_str = "";
    delay(1000); // delay needed to wait for next transmission
  }
  if (r == LOX_MAX_READINGS_ATTEMPTS){
    return 0; // Failed to read stream
  }
  else {
    return 1; // Read stream OK
  }
}


/**
 * Initialyze LOX O2 sensor
 * User implementation: Setup Serial interface for the sensor
 */
int initLoxDevice(void){
  float o2_pp=0, o2_percent=0, temp=0, press=0;
  SerialN2.begin(LOX_SERIAL_SPEED);
  pinPeripheral(GPIO_SERIAL_2_RX, PIO_SERCOM); //Assign RX function to pin 1
  pinPeripheral(GPIO_SERIAL_2_TX, PIO_SERCOM); //Assign TX function to pin 0
  SerialN2.write("M 0\r\n"); // Set sensor to streaming mode
  delay(1000);
  // Try to read the sensor
  if(readLoxStream(&o2_pp, &o2_percent, &temp, &press)){
    return 1;
  }
  else {
    return 0;
  }
}


/**
 * Execute a reading from each sensor
 * Update the values in the global variables
 */
void dataAcquisition(void){
  float temp_o2_pp=0, temp_o2=0, temp_temp=0, temp_press=0;
  readLoxStream(&temp_o2_pp, &temp_o2, &temp_temp, &temp_press);
  if (temp_o2 > 0){
    global_o2_pp = temp_o2_pp;
    global_o2 = temp_o2;
    global_gas_temp = temp_temp;
    global_gas_press = temp_press;
  }
  int co2_read = readCozirStream();
  if (co2_read > 0){
    global_co2 = (float)co2_read/100;
  }
}


/**
 * Update the array
 */
void updateDataArray(float *data_arr, int size, float value) {
  for (int i = 0; i < size - 1; i++) {
    data_arr[i] = data_arr[i + 1];
  }
  data_arr[size - 1] = value;
}



/**
 * Event Handler control
 * Controls the LED activiation for each state
 */
int eventHandler(int ev_code){
  // timerDisableTC3(); // Removed timer from LED control
  digitalWrite(GPIO_LED_GREEN, LOW);
  digitalWrite(GPIO_LED_RED, LOW);
  switch (ev_code) {
    case EVENT_CODE_CRITICAL_ERROR: // LOCK THE CPU
      for(;;){ 
        digitalWrite(GPIO_LED_RED, HIGH);
        delay(100);
        digitalWrite(GPIO_LED_RED, LOW);
        delay(100);
      }
      break;
    case EVENT_CODE_STARTUP_ERROR: // LOCK THE CPU
      for(;;){
        digitalWrite(GPIO_LED_RED, HIGH);
        digitalWrite(GPIO_LED_GREEN, LOW);
        delay(100);
        digitalWrite(GPIO_LED_RED, LOW);
        digitalWrite(GPIO_LED_GREEN, HIGH);
        delay(100);
      }
      break;
    case EVENT_CODE_SETUP:
      digitalWrite(GPIO_LED_RED, HIGH);
      break;
    case EVENT_CODE_RUNNING:
      digitalWrite(GPIO_LED_GREEN, HIGH);
      break;
    case EVENT_CODE_AP_CONN:
      digitalWrite(GPIO_LED_RED, HIGH);
      digitalWrite(GPIO_LED_GREEN, HIGH);
      // timerEnableTC3(TIMER_INTERVAL_100_MS); // Removed timer from LED control
      break;
    default:
      return 0;
  }
  return 1;
}


/**
 *
 */
void htmlGetRequestHandler(String req){
    if (req.indexOf("GET / ") >= 0) {
      handleRoot(client);
    }
    else if (req.indexOf("GET /data") >= 0) {
      handleSensorData(client);
    }
    else if (req.indexOf("GET /sensor_values") >= 0) {
      handleSensorValues(client);
    }
    else if (req.indexOf("GET /control") >= 0) {
      handleMotorCtrl(client);
    }
    else if (req.indexOf("GET /m1p00") >= 0) {
      pwmMotorCtrl(PWM_CTRL_POWER_0, GPIO_PWM_MOTOR_1);
      handleMotorCtrl(client);
    }
    else if (req.indexOf("GET /m1pff") >= 0) {
      pwmMotorCtrl(PWM_CTRL_POWER_100, GPIO_PWM_MOTOR_1);
      handleMotorCtrl(client);
    }
    else if (req.indexOf("GET /m1p50") >= 0) {
      pwmMotorCtrl(PWM_CTRL_POWER_50, GPIO_PWM_MOTOR_1);
      handleMotorCtrl(client);
    }
    else if (req.indexOf("GET /m1p20") >= 0) {
      pwmMotorCtrl(PWM_CTRL_POWER_20, GPIO_PWM_MOTOR_1);
      handleMotorCtrl(client);
    }
    else if (req.indexOf("GET /m1p80") >= 0) {
      pwmMotorCtrl(PWM_CTRL_POWER_80, GPIO_PWM_MOTOR_1);
      handleMotorCtrl(client);
    }
    else {
      handleNotFound(client);
    }

}



/**
 * Timer setup
 * Enable Timer TC3
 */
void timerEnableTC3(int interval_ms) {
  // Enable generic clock for TC3 timer
  // TC3 shares its clock with TCC2
  // GCLK_CLKCTRL_ID_TCC2_TC3 is the correct ID for both on SAMD21.
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCLK_CLKCTRL_ID_TCC2_TC3) |
                      GCLK_CLKCTRL_GEN_GCLK0 |
                      GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Disable TC3 before configuring
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

  // Configure TC3: 16-bit, match frequency, 1024 prescaler
  TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |
                           TC_CTRLA_WAVEGEN_MFRQ |
                           TC_CTRLA_PRESCALER_DIV1024;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

  // Set clock divider
  // clk_div = (48,000,000 / 1024) × (time in seconds)
  //TC3->COUNT16.CC[0].reg = clk_div;
  TC3->COUNT16.CC[0].reg = (interval_ms * (48000000 / 1024)) / 1000;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

  // Enable interrupt and set handler
  NVIC_EnableIRQ(TC3_IRQn);
  TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;

  // Enable TC3 timer
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}


/**
 * Timer setup
 * Disable Timer TC3
 */
void timerDisableTC3() {
  // Disable TC3 timer
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

  // Disable interrupt
  TC3->COUNT16.INTENCLR.reg = TC_INTENCLR_MC0;
  NVIC_DisableIRQ(TC3_IRQn);
}

/* ISR Handlers */

void TC3_Handler() {
  // Clear interrupt flag
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;

  // digitalWrite(GPIO_LED_GREEN, !digitalRead(GPIO_LED_GREEN)); // Toggle green led
}

void SERCOM3_Handler()
{
  SerialN2.IrqHandler();
}


/**
 * PROGRAM SETUP
 */
void setup() {
  int set_up_error_flag = 0;
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GPIO_RELAY_1, OUTPUT);
  pinMode(GPIO_LED_RED, OUTPUT);
  pinMode(GPIO_LED_GREEN, OUTPUT);
  pinMode(GPIO_SENSOR_CALIBRATION, INPUT_PULLUP);
  pinMode(GPIO_PWM_MOTOR_1, OUTPUT);
  pinMode(GPIO_SENSOR_PWR_CTRL, OUTPUT);

  Serial.begin(115200);
  delay(1000); // Delay for debugging
  global_state = FSM_SETUP; // TODO Change global_state to private type (state)
  eventHandler(EVENT_CODE_SETUP);
  Serial.print("\n-->Software Version: ");
  Serial.print(SW_VERSION);
  Serial.print("\n-->Gas Analyzer Control: SETUP");

  digitalWrite(GPIO_SENSOR_PWR_CTRL, HIGH); // Power on the sensors

  if (!initCozirDevice()){
    Serial.print("\n-->Gas Analyzer Control: Cozir Start FAILED");
    set_up_error_flag = 1;
  }

  if (!initLoxDevice()){
    Serial.print("\n-->Gas Analyzer Control: LoxO2 Start FAILED");
    set_up_error_flag = 1;
  }

  if(digitalRead(GPIO_SENSOR_CALIBRATION) == LOW){ // Calibrate sensors
    //eventHandler(EVENT_CODE_CALIBRATION);
    if (!calibrateCozirDevice()){
      Serial.print("\n-->Gas Analyzer Control: Cozir Calibration FAILED");
      set_up_error_flag = 1;
    }
    else{
      Serial.print("\n-->Gas Analyzer Control: Cozir Calibration OK");
      eventHandler(EVENT_CODE_SETUP); // Resume setup
    }
  }

  pwmMotorCtrl(PWM_CTRL_POWER_0, GPIO_PWM_MOTOR_1); // Start with motor 1 off
  relayMotorCtrl(GPIO_PWM_MOTOR_1, GPIO_RELAY_1, RELAY_CTRL_PWR_ON); // Connect Relay 1
  Serial.print("\n-->Gas Analyzer Control: Relay UP at pin D");
  Serial.print(GPIO_RELAY_1);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.print("\n-->Gas Analyzer WiFi: Communication with WiFi module FAILED");
    set_up_error_flag = 1;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.print("\n-->Gas Analyzer WiFi: Firmware update needed. Current FW: ");
    Serial.print(WIFI_FIRMWARE_LATEST_VERSION);
  }

  eventHandler(EVENT_CODE_AP_CONN);
  // Create open network:
  WiFi.config(IPAddress(10, 0, 0, 1));
  statusWiFi = WiFi.beginAP(ssid, pass);
  Serial.print("\n-->Gas Analyzer WiFi: WiFi.begin return value: ");
  Serial.print(statusWiFi);
  if (statusWiFi != WL_AP_LISTENING) {
    Serial.print("\n-->Gas Analyzer WiFi: Creating access point FAILED");
    set_up_error_flag = 1;
  }
  else {
    Serial.print("\n-->Gas Analyzer WiFi: Creating access point OK");
  }

  if (set_up_error_flag){
    eventHandler(EVENT_CODE_STARTUP_ERROR);
  }
  // wait for connections
  delay(1000);
  // start the web server on port 80
  server.begin();
  printWiFiStatus();


}


/**
 * PROGRAM LOOP
 */
void loop() {
  static int loop_count = 0;
  if(global_state == FSM_SETUP){
    global_state = FSM_AP_SERVER; // update state
    eventHandler(EVENT_CODE_RUNNING);
  }
  client = server.available();
  if (client) {
    loop_count++;
    String req = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        req += c; // Append request
        if (req.endsWith("\r\n\r\n")) {
          break;
        }
      }
    }
    htmlGetRequestHandler(req); // reply the client request
    dataAcquisition(); // read the sensors
    delay(1);
    client.stop();
  }
}

/**
 * HTML handlers
 *
 */
void handleRoot(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<!DOCTYPE html><html><body>");

  client.println("<h2 style='font-size:48px; margin-bottom:20px;'>GAS ANALYZER CONTROL PANEL</h2>");

  client.println("<a href=\"/data\" style='"
                 "display:inline-block; "
                 "font-size:48px; "
                 "margin-bottom:20px; "
                 "padding:20px 40px; "
                 "background-color:#4CAF50; "
                 "color:white; "
                 "text-decoration:none; "
                 "border-radius:10px;'>SENSOR DATA</a><br>");

  client.println("<a href=\"/control\" style='"
                 "display:inline-block; "
                 "font-size:48px; "
                 "margin-top:20px; "
                 "padding:20px 40px; "
                 "background-color:#2196F3; "
                 "color:white; "
                 "text-decoration:none; "
                 "border-radius:10px;'>MOTOR CONTROL</a>");

  client.print("<p style='font-size:24px; margin-top:80px;'>Software Version: ");
  client.print(SW_VERSION);
  client.println("</p>");
  client.print("<p style='font-size:24px; margin-top:20px;'>Last update: ");
  client.print(SW_DATE);
  client.println("</p>");

  client.println("</body></html>");
}

void handleSensorValues(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Access-Control-Allow-Origin: *");
  client.println();

  client.print("{\"co2\":");
  client.print(global_co2);
  client.print(",\"o2\":");
  client.print(global_o2);
  client.print(",\"o2_pp\":");
  client.print(global_o2_pp);
  client.print(",\"gasTemp\":");
  client.print(global_gas_temp);
  client.print(",\"gasPress\":");
  client.print(global_gas_press);
  client.println("}");
}

void handleSensorData(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<!DOCTYPE html><html><head>");
  client.println("<meta charset='UTF-8'>");
  client.println("<title>Sensor Data</title>");
  client.println("<script>");
  client.println("function updateSensorData() {");
  client.println("  fetch('/sensor_values')");
  client.println("    .then(response => response.json())");
  client.println("    .then(data => {");
  client.println("      document.getElementById('co2').innerText = data.co2 + ' %';");
  client.println("      document.getElementById('o2').innerText = data.o2 + ' %';");
  client.println("      document.getElementById('o2_pp').innerText = data.o2_pp + ' mbar';");
  client.println("      document.getElementById('gasTemp').innerText = data.gasTemp + ' °C';");
  client.println("      document.getElementById('gasPress').innerText = data.gasPress + ' mbar';");
  client.println("    });");
  client.println("}");
  client.println("setInterval(updateSensorData, 1000);");
  client.println("</script>");
  client.println("</head><body>");

  client.println("<h2 style='font-size:48px; margin-bottom:30px;'>SENSOR DATA</h2>");

  client.println("<p style='font-size:48px; margin-top:40px; margin-bottom:20px;'>CO2: <span id='co2'>--</span></p>");
  client.println("<p style='font-size:48px; margin-bottom:30px;'>O2: <span id='o2'>--</span></p>");
  client.println("<p style='font-size:48px; margin-bottom:30px;'>O2 PP: <span id='o2_pp'>--</span></p>");
  client.println("<p style='font-size:48px; margin-bottom:30px;'>Gas Temperature: <span id='gasTemp'>--</span></p>");
  client.println("<p style='font-size:48px; margin-bottom:30px;'>Gas Pressure: <span id='gasPress'>--</span></p>");

  client.println("<a href=\"/\" style='"
                 "display:inline-block; "
                 "font-size:48px; "
                 "padding:20px 40px; "
                 "margin-top:20px; "
                 "background-color:#555; "
                 "color:white; "
                 "text-decoration:none; "
                 "border-radius:10px;'>"
                 "&#8592; Back to Home</a>");

  client.println("</body></html>");
}


void handleMotorCtrl(WiFiClient &client) {
  int round_motor_1_pwr = (int)global_motor_1_pwr;
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<!DOCTYPE html><html><body>");

  client.println("<h2 style='font-size:48px; margin-bottom:30px;'>MOTOR CONTROL</h2>");

  client.print("<p style='font-size:48px; margin-bottom:30px;'>MOTOR1 AT [");
  client.print(round_motor_1_pwr);
  client.println("%] PWR</p>");

  client.println("<a href=\"/m1p00\" style='"
                 "display:inline-block; "
                 "font-size:48px; "
                 "margin-bottom:20px; "
                 "padding:20px 40px; "
                 "background-color:#2196F3; "  // blue
                 "color:white; "
                 "text-decoration:none; "
                 "border-radius:10px;'>MOTOR1 OFF</a><br>");

  client.println("<a href=\"/m1pff\" style='"
                 "display:inline-block; "
                 "font-size:48px; "
                 "margin-bottom:20px; "
                 "padding:20px 40px; "
                 "background-color:#f44336; "  // red
                 "color:white; "
                 "text-decoration:none; "
                 "border-radius:10px;'>MOTOR1 TO 100%  PWR</a><br>");

    client.println("<a href=\"/m1p80\" style='"
                 "display:inline-block; "
                 "font-size:48px; "
                 "margin-bottom:20px; "
                 "padding:20px 40px; "
                 "background-color:#A44A3F; "  // light red
                 "color:white; "
                 "text-decoration:none; "
                 "border-radius:10px;'>MOTOR1 TO 80%  PWR</a><br>");

  client.println("<a href=\"/m1p50\" style='"
                 "display:inline-block; "
                 "font-size:48px; "
                 "margin-bottom:20px; "
                 "padding:20px 40px; "
                 "background-color:#FF9800; "  // orange
                 "color:white; "
                 "text-decoration:none; "
                 "border-radius:10px;'>MOTOR1 TO 50%  PWR</a><br>");

  client.println("<a href=\"/m1p20\" style='"
                 "display:inline-block; "
                 "font-size:48px; "
                 "margin-bottom:20px; "
                 "padding:20px 40px; "
                 "background-color:#FFB86F; "  // light orange
                 "color:white; "
                 "text-decoration:none; "
                 "border-radius:10px;'>MOTOR1 TO 20%  PWR</a><br>");

  client.println("<a href=\"/\" style='"
                 "display:inline-block; "
                 "font-size:48px; "
                 "margin-top:40px; "
                 "padding:20px 40px; "
                 "background-color:#555; "
                 "color:white; "
                 "text-decoration:none; "
                 "border-radius:10px;'>"
                 "&#8592; Back to Home</a>");

  client.println("</body></html>");
}

void handleNotFound(WiFiClient &client) {
  client.println("HTTP/1.1 404 Not Found");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<h1>404 - Not Found</h1>");
}

// Wifi status
void printWiFiStatus(void) {
  Serial.print("\nSSID: ");
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.print("GAS ANALYZER CONTROL PANEL AT http://");
  Serial.println(ip);

}
