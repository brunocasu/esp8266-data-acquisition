/*

 */

#include "Arduino.h"
#include <SPI.h>
#include <WiFiNINA.h>
#include <cozir.h>

/* DEFINES */
#define AP_SSID "GAS_ANALYZER"
#define AP_PASS "gamkr1010"

#define GPIO_PWM_MOTOR_1        2 // Function D2
#define GPIO_SENSOR_CALIBRATION 5 // Function D5
#define GPIO_LED_GREEN          6 // Function D6
#define GPIO_LED_RED            7 // Function D7
#define GPIO_RELAY_1            8 // Function D8
#define GPIO_SENSOR_PWR_CTRL    9 // Function D9

#define EVENT_CODE_CRITICAL_ERROR 1
#define EVENT_CODE_STARTUP_ERROR  2
#define EVENT_CODE_CALIBRATION    3
#define EVENT_CODE_SETUP          4
#define EVENT_CODE_RUNNING        5
#define EVENT_CODE_CONNECTION     6

#define RELAY_CTRL_CRIT_SHUT_OFF    0
#define RELAY_CTRL_NORMAL_SHUT_OFF  1
#define RELAY_CTRL_PWR_ON           2

#define PWM_CTRL_POWER_0            0   // motor off
#define PWM_CTRL_POWER_1            1   // 100% power
#define PWM_CTRL_POWER_2            2   // 50% power
#define PWM_CTRL_POWER_5            5   // 20% power
#define PWM_CTRL_POWER_10           10  // 10% power

#define COZIR_MAX_READINGS_ATTEMPTS 5
#define TIMER_CLK_DIV_100_MS_INTERVAL 4688
#define TIMER_CLK_DIV_800_MS_INTERVAL 37500

#define FSM_RESET     0
#define FSM_SETUP     1
#define FSM_AP_SERVER 2

#define DEBUG_MODE

/* PV */
int global_state = FSM_RESET;

char ssid[] = AP_SSID;  // Network SSID
char pass[] = AP_PASS;  // Network password (use for WPA, or use as key for WEP)
int keyIndex = 0;       // Network key index number (needed only for WEP)

COZIR czr(&Serial1);  // CO2 sensor handler
float read_co2=0;     // CO2 reading (in %)

int statusWiFi = WL_IDLE_STATUS;
WiFiServer server(80);
WiFiClient client;

float global_co2 = 0;
float global_o2 = 0;

/* PFD */
int parseCozirStream(String input);
int readCozirStream(void);
int initCozirDevice(void);
int calibrateCozirDevice(void);
void pwmMotorCtrl(int div, int motor_ctrl_pin);
void relayMotorCtrl(int motor_ctrl_pin, int relay_pin, int op);
void handleRoot(WiFiClient &client);
void handleSensorData(WiFiClient &client);
void handleMotorCtrl(WiFiClient &client);
void handleNotFound(WiFiClient &client);
void printWiFiStatus(void);

/* PF */
/**
 * Control function for PWM for connected motors
 * div: duty cycle division (e.g. div=4, output avg_voltage = max_voltage/4)
 * if div = 0, Shut down motor
 * motor_ctrl_pin: GPIO to be controlled
 */
void pwmMotorCtrl(int div, int motor_ctrl_pin){
  if(div == 0){
    analogWrite(motor_ctrl_pin, 0); // Power off motor
  }
  else{
    analogWrite(motor_ctrl_pin, 255 / div); // Setup PWM
  }
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
 * Set the Cozir sensor to Streaming mode and reads the first full string sent
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
    Serial.print(currentChar);
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
    Serial.println(result);
#endif
    sensorValue = parseCozirStream(result);
  }
#ifdef DEBUG_MODE
  Serial.print("-->Cozir parsed sensor value:");
  Serial.println(sensorValue);
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
 * The sensor must be calibrated ONLY WHEN SUBMITTED TO ATMOSPHERIC AIR
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
 *
 */
int initCozirDevice(void){
  // In Arduino MKR WiFi 1010, Serial1 uses pins D14(Controller TX) and D13(Controller RX)
  Serial1.begin(38400); // SprintIR-R-100 uses 38400 baud rate - SprintIR-WF-100 uses 9600 baud rate
  czr.init();
  // Set sensor to STREAMING MODE (continuous write to serial)
  czr.setOperatingMode(CZR_STREAMING);
  // After sending set operating mode the serial hangs - serial listener must detect first Z sent.
  delay(50);
  // Test reading the sensor
  if (readCozirStream() > 0){
    return 1;
  }
  else {
    // Attempt to restart the sensor
    digitalWrite(GPIO_SENSOR_PWR_CTRL, LOW); // Power off the sensors
    delay(50);
    digitalWrite(GPIO_SENSOR_PWR_CTRL, HIGH); // Power on the sensors
    delay(500);
    if (readCozirStream() > 0){
      return 1;
    }
    else {
      return 0;
    }
  }
}


/**
 * Event Handler control
 * Controls the LED activiation for each state
 */
int eventHandler(int ev_code){
  timerDisable();
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
    case EVENT_CODE_CALIBRATION:
      digitalWrite(GPIO_LED_RED, HIGH);
      digitalWrite(GPIO_LED_GREEN, HIGH);
      break;
    case EVENT_CODE_SETUP:
      digitalWrite(GPIO_LED_RED, HIGH);
      break;
    case EVENT_CODE_RUNNING:
      digitalWrite(GPIO_LED_GREEN, HIGH);
      break;
    case EVENT_CODE_CONNECTION:
      timerEnable(TIMER_CLK_DIV_100_MS_INTERVAL);
    default:
      return 0;
  }
}


/**
 * Event Handler
 * Controls the LED activiation for each state
 */
void timerEnable(int clk_div) {
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
  TC3->COUNT16.CC[0].reg = clk_div;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

  // Enable interrupt and set handler
  NVIC_EnableIRQ(TC3_IRQn);
  TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;

  // Enable TC3 timer
  TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
}

void timerDisable() {
  // Disable TC3 timer
  TC3->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

  // Disable interrupt
  TC3->COUNT16.INTENCLR.reg = TC_INTENCLR_MC0;
  NVIC_DisableIRQ(TC3_IRQn);
}

void TC3_Handler() {
  // Clear interrupt flag
  TC3->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;

  digitalWrite(GPIO_LED_GREEN, !digitalRead(GPIO_LED_GREEN)); // Toggle green led
}


/**
 * PROGRAM SETUP
 */
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GPIO_RELAY_1, OUTPUT);
  pinMode(GPIO_LED_RED, OUTPUT);
  pinMode(GPIO_LED_GREEN, OUTPUT);
  pinMode(GPIO_SENSOR_CALIBRATION, INPUT_PULLUP);
  pinMode(GPIO_PWM_MOTOR_1, OUTPUT);
  pinMode(GPIO_SENSOR_PWR_CTRL, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  global_state = FSM_SETUP; // TODO Change global_state to private type (state)
  eventHandler(EVENT_CODE_SETUP);
  Serial.print("\n-->Gas Analyzer Control: SETUP");

  digitalWrite(GPIO_SENSOR_PWR_CTRL, HIGH); // Power on the sensors
  if (!initCozirDevice()){
    Serial.print("\n-->Gas Analyzer Control: Cozir Start FAILED");
    eventHandler(EVENT_CODE_STARTUP_ERROR);
  }
  if(digitalRead(GPIO_SENSOR_CALIBRATION) == LOW){ // Calibrate sensors
    eventHandler(EVENT_CODE_CALIBRATION);
    if (!calibrateCozirDevice()){
      Serial.print("\n-->Gas Analyzer Control: Cozir Calibration FAILED");
      eventHandler(EVENT_CODE_STARTUP_ERROR);
    }
    else{
      Serial.print("\n-->Gas Analyzer Control: Cozir Calibration OK");
      eventHandler(EVENT_CODE_SETUP); // Resume setup
    }
  }
  Serial.print("\n-->Gas Analyzer Control: Cozir Start OK");

  // Connect the relay on the motors
  pwmMotorCtrl(PWM_CTRL_POWER_0, GPIO_PWM_MOTOR_1); // Start with motor 1 off
  relayMotorCtrl(GPIO_PWM_MOTOR_1, GPIO_RELAY_1, RELAY_CTRL_PWR_ON);
  Serial.print("\n-->Gas Analyzer Control: Relay UP n. ");
  Serial.print(GPIO_RELAY_1);


  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.print("\n-->Gas Analyzer WiFi: Communication with WiFi module FAILED");
    eventHandler(EVENT_CODE_STARTUP_ERROR);
    // don't continue
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.print("\n-->Gas Analyzer WiFi: Firmware update needed. Current FW: ");
    Serial.print(WIFI_FIRMWARE_LATEST_VERSION);
  }

  WiFi.config(IPAddress(10, 0, 0, 1));
  eventHandler(EVENT_CODE_CONNECTION);
  // print the network name (SSID);
  Serial.print("\n-->Gas Analyzer WiFi: Creating access point named: ");
  Serial.print(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  statusWiFi = WiFi.beginAP(ssid, pass);
  if (statusWiFi != WL_AP_LISTENING) {
    Serial.print("\n-->Gas Analyzer WiFi: Creating access point FAILED");
    eventHandler(EVENT_CODE_STARTUP_ERROR);
  }
  Serial.print("\n-->Gas Analyzer WiFi: Creating access point OK");
  // wait 10??? seconds for connection:
  delay(5000);
  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}


void loop() {
  if(global_state == FSM_SETUP){
    global_state = FSM_AP_SERVER; // update state
    eventHandler(EVENT_CODE_RUNNING);
  }
  client = server.available();
  if (client) {
    Serial.println("Client connected");
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

    // Route handling based on request
    if (req.indexOf("GET / ") >= 0) {
      handleRoot(client);
    }
    else if (req.indexOf("GET /data") >= 0) {
      handleSensorData(client);
    }
    else if (req.indexOf("GET /control") >= 0) {
      handleMotorCtrl(client);
    }
    else if (req.indexOf("GET /m1p00") >= 0) {
      pwmMotorCtrl(PWM_CTRL_POWER_0, GPIO_PWM_MOTOR_1);
      handleMotorCtrl(client);
    }
    else if (req.indexOf("GET /m1pff") >= 0) {
      pwmMotorCtrl(PWM_CTRL_POWER_1, GPIO_PWM_MOTOR_1);
      handleMotorCtrl(client);
    }
    else if (req.indexOf("GET /m1p50") >= 0) {
      pwmMotorCtrl(PWM_CTRL_POWER_2, GPIO_PWM_MOTOR_1);
      handleMotorCtrl(client);
    }
    else if (req.indexOf("GET /m1p20") >= 0) {
      pwmMotorCtrl(PWM_CTRL_POWER_5, GPIO_PWM_MOTOR_1);
      handleMotorCtrl(client);
    }
    else if (req.indexOf("GET /m1p10") >= 0) {
      pwmMotorCtrl(PWM_CTRL_POWER_10, GPIO_PWM_MOTOR_1);
      handleMotorCtrl(client);
    }
    else {
      handleNotFound(client);
    }
    delay(1);
    global_co2 += 0.1; // REMOVE
    global_o2 += 0.1;  // REMOVE
    client.stop();
  }
}

/**
 *
 *
 */
void handleRoot(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<!DOCTYPE html><html><body>");

  client.println("<h2 style='font-size:48px; margin-bottom:20px;'>Welcome to MKR Web Server</h2>");

  client.println("<a href=\"/data\" style='"
                 "display:inline-block; "
                 "font-size:48px; "
                 "margin-bottom:20px; "
                 "padding:20px 40px; "
                 "background-color:#4CAF50; "
                 "color:white; "
                 "text-decoration:none; "
                 "border-radius:10px;'>View Sensor Data</a><br>");

  client.println("<a href=\"/control\" style='"
                 "display:inline-block; "
                 "font-size:48px; "
                 "margin-top:20px; "
                 "padding:20px 40px; "
                 "background-color:#2196F3; "
                 "color:white; "
                 "text-decoration:none; "
                 "border-radius:10px;'>Toggle Motor</a>");

  client.println("</body></html>");
}

void handleSensorData(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<!DOCTYPE html><html><body>");
  client.println("<h2>Sensor Data</h2>");
  client.print("<p>CO2: ");
  client.print(global_co2);
  client.println(" %</p>");
  client.print("<p> O2: ");
  client.print(global_o2);
  client.println(" %</p>");
  client.println("<p><a href=\"/\">&#8592; Back to Home</a></p>");
  client.println("</body></html>");
}

void handleMotorCtrl(WiFiClient &client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<!DOCTYPE html><html><body>");

  client.println("<h2 style='font-size:48px; margin-bottom:20px;'>MOTOR CONTROL</h2>");

  client.println("<p style='font-size:48px; margin-bottom:20px;'><a href=\"/m1p00\">MOTOR1 OFF</a></p>");
  client.println("<p style='font-size:48px; margin-bottom:20px;'><a href=\"/m1pff\">MOTOR1 TO 100% PWR</a></p>");
  client.println("<p style='font-size:48px; margin-bottom:20px;'><a href=\"/m1p50\">MOTOR1 TO 50% PWR</a></p>");
  client.println("<p style='font-size:48px; margin-bottom:20px;'><a href=\"/m1p20\">MOTOR1 TO 20% PWR</a></p>");
  client.println("<p style='font-size:48px; margin-bottom:20px;'><a href=\"/m1p10\">MOTOR1 TO 10% PWR</a></p>");
  client.println("<p style='font-size:48px; margin-bottom:20px;'><a href=\"/\">&#8592; Back to Home</a></p>");

  client.println("</body></html>");
}

void handleNotFound(WiFiClient &client) {
  client.println("HTTP/1.1 404 Not Found");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<h1>404 - Not Found</h1>");
}


void printWiFiStatus(void) {
  // print the SSID of the network you're attached to:
  Serial.print("\nSSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}
