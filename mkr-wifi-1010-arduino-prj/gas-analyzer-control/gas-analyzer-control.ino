/*

 */

#include "Arduino.h"
#include <SPI.h>
#include <WiFiNINA.h>
#include <cozir.h>

/* DEFINES */
#define AP_SSID "gas_analyzer_mkr1010"
#define AP_PASS "gamkr1010"

#define GPIO_PWM_MOTOR_1        2 // Function D2
#define GPIO_SENSOR_CALIBRATION 5 // Function D5
#define GPIO_LED_GREEN          6 // Function D6
#define GPIO_LED_RED            7 // Function D7
#define GPIO_RELAY_1            8 // Function D8
#define GPIO_SENSOR_PWR_CTRL    9 // Function D9

#define EVENT_CODE_CRITICAL_ERROR 1
#define EVENT_CODE_CALIBRATION    2
#define EVENT_CODE_SETUP          3
#define EVENT_CODE_RUNNING        4
#define EVENT_CODE_CONNECTION     5

#define RELAY_CTRL_CRIT_SHUT_OFF    0
#define RELAY_CTRL_NORMAL_SHUT_OFF  1
#define RELAY_CTRL_PWR_ON           2

#define COZIR_MAX_READINGS_ATTEMPTS 10
#define TIMER_CLK_DIV_100_MS_INTERVAL 4688
#define TIMER_CLK_DIV_800_MS_INTERVAL 37500

/* PV */
char ssid[] = AP_SSID;  // Network SSID
char pass[] = AP_PASS;  // Network password (use for WPA, or use as key for WEP)
int keyIndex = 0;       // Network key index number (needed only for WEP)

COZIR czr(&Serial1);  // CO2 sensor handler
float read_co2=0;     // CO2 reading (in %)

int statusWiFi = WL_IDLE_STATUS;
WiFiServer server(80);

/* PFD */
int parseCozirStream(String input);
int readCozirStream(void);
int initCozirDevice(void);
void calibrateCozirDevice(void);
void pwmMotorCtrl(int div, int motor_ctrl_pin);
void relayMotorControl(int motor_ctrl_pin, int relay_pin, int op);


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
void relayMotorControl(int motor_ctrl_pin, int relay_pin, int op){
  switch (op) {
    case RELAY_CTRL_CRIT_SHUT_OFF:
      // WARNING! In CRITICAL MOTOR SHUT DOWN the MCU might be restarted
      digitalWrite(motor_ctrl_pin, LOW); // Power off motor via MOSFET
      digitalWrite(relay_pin, LOW); // Cut off VCC for the motor via RELAY
      break;
    case  RELAY_CTRL_NORMAL_SHUT_OFF: // Graceful motor shut down
      digitalWrite(motor_ctrl_pin, LOW); // First Power off motor via MOSFET
      delay(500); // Delay added to prevent voltage drop when switching the motor
      digitalWrite(relay_pin, LOW); // Cut off VCC for the motor via RELAY
      break;
    case RELAY_CTRL_PWR_ON: // Motor start
      if (digitalRead(relay_pin) == LOW){
        digitalWrite(motor_ctrl_pin, LOW); // Power off motor via MOSFET
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
 * This function parses the inut string to retrieve the CO2 measurement
 * The Cozir sensor on Streaming mode sends characters as the example: "Z 00026"
 * This function returns the numerical value (int) of the parsed input string
 * The value, when multipled by 100, corrsponds to the CO2 measurement in ppm
 */
int parseCozirStream(String input) {
  // Find the position of 'Z' in the input string
  int zPos = input.indexOf('Z');
  int sensorValue = 0;
  if (zPos != -1) {
    // Extract the substring starting from 'Z' position
    String numberString = input.substring(zPos + 2);
    numberString.trim();
    sensorValue = numberString.toInt();
    return sensorValue;
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
        Serial.print(currentChar); // Debug
        currentChar = Serial1.read();
        result += currentChar;
      }
    }
    k++;
    Serial.print("\n-->Cozir string read: ");
    Serial.println(result);
    sensorValue = parseCozirStream(result);
  }
  Serial.print("-->Cozir parsed sensor value:");
  Serial.println(sensorValue);
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
void calibrateCozirDevice(void){
  czr.setOperatingMode(CZR_POLLING);
  delay(100);
  Serial.println(czr.calibrateFreshAir());
  delay(100);
}


/**
 * Initialyze Cozir sensor
 *
 */
int initCozirDevice(void){
  // In Arduino MKR WiFi 1010, Serial1 uses pins D14(Controller TX) and D13(Controller RX)
  Serial1.begin(38400); // SprintIR-R-100 uses 38400 baud rate - SprintIR-WF-100 uses 9600 baud rate
  czr.init();
  if(digitalRead(GPIO_SENSOR_CALIBRATION) == LOW){
    eventHandler(EVENT_CODE_CALIBRATION);
    calibrateCozirDevice();
    Serial.println("-->Cozir Start CALIBRATION");
  }

  // Set sensor to STREAMING MODE (continuous write to serial)
  czr.setOperatingMode(CZR_STREAMING);
  // After sending set operating mode the serial hangs - serial listener must detect first Z sent.
  delay(10);
  // Test reading the sensor
  if (readCozirStream() > 0){
    Serial.println("-->Cozir Start OK");
    return 1;
  }
  else
    Serial.println("-->Cozir Start FAILED"); 
    return 0;
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

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GPIO_RELAY_1, OUTPUT);
  pinMode(GPIO_LED_RED, OUTPUT);
  pinMode(GPIO_LED_GREEN, OUTPUT);
  pinMode(GPIO_SENSOR_CALIBRATION, INPUT_PULLUP);
  pinMode(GPIO_PWM_MOTOR_1, OUTPUT);
  pinMode(GPIO_SENSOR_PWR_CTRL, OUTPUT);

  eventHandler(EVENT_CODE_SETUP);
  Serial.println("-->Gas Analyzer Control: SETUP");
  Serial.begin(115200);
  delay(5000); // REMOVE

  eventHandler(EVENT_CODE_CONNECTION);
  Serial.println("-->Gas Analyzer Control: CONNECTING");
  relayMotorControl(GPIO_PWM_MOTOR_1, GPIO_RELAY_1, RELAY_CTRL_PWR_ON);
  pwmMotorCtrl(4, GPIO_PWM_MOTOR_1);
  delay(3000);
  pwmMotorCtrl(0, GPIO_PWM_MOTOR_1);
  delay(3000);
  pwmMotorCtrl(2, GPIO_PWM_MOTOR_1);
  delay(3000);
  pwmMotorCtrl(0, GPIO_PWM_MOTOR_1);
  delay(3000);
  //pwmMotorCtrl(0, GPIO_PWM_MOTOR_1);
  //delay(1000);
  relayMotorControl(GPIO_PWM_MOTOR_1, GPIO_RELAY_1, RELAY_CTRL_CRIT_SHUT_OFF);


  // Initialyze serial for Cozir sensor
  digitalWrite(GPIO_SENSOR_PWR_CTRL, HIGH); // Power on the sensors
  if (!initCozirDevice()){eventHandler(EVENT_CODE_CRITICAL_ERROR);}
  
  Serial.println("-->Gas Analyzer Control: RUNNING");
  eventHandler(EVENT_CODE_RUNNING);
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    eventHandler(EVENT_CODE_CRITICAL_ERROR);
    // don't continue
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  statusWiFi = WiFi.beginAP(ssid, pass);
  if (statusWiFi != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10??? seconds for connection:
  delay(5000);
  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}



void loop() {

  // compare the previous statusWiFi to the current status
  if (statusWiFi != WiFi.status()) {
    // it has changed update the variable
    statusWiFi = WiFi.status();

    if (statusWiFi == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
  
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      delayMicroseconds(10);                // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out to the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> turn the LED on<br>");
            client.print("Click <a href=\"/L\">here</a> turn the LED off<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(LED_BUILTIN, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(LED_BUILTIN, LOW);                // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}
