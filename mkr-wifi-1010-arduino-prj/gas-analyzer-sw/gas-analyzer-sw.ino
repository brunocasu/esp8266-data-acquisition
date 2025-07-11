/*
  WiFi Web Server LED Blink

  A simple web server that lets you blink an LED via the web.
  This sketch will create a new access point (with no password).
  It will then launch a new server and print out the IP address
  to the Serial Monitor. From there, you can open that address in a web browser
  to turn on and off the LED on pin 13.

  If the IP address of your board is yourAddress:
    http://yourAddress/H turns the LED on
    http://yourAddress/L turns it off

  created 25 Nov 2012
  by Tom Igoe
  adapted to WiFi AP by Adafruit
 */

#include "Arduino.h"
#include <SPI.h>
#include <WiFiNINA.h>
#include <cozir.h>
//#include "arduino_secrets.h"

#define AP_SSID "gas_analyzer_mkr1010"
#define AP_PASS "gamkr1010"
#define GPIO_LED_GREEN  6 // Function D6
#define GPIO_LED_RED  7 // Function D7
#define GPIO_RELAY_CTRL 8 // Function D8

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = AP_SSID;        // your network SSID (name)
char pass[] = AP_PASS;    // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                // your network key index number (needed only for WEP)

// Cozir CO2 sensor
COZIR czr(&Serial1);
float read_co2=0;


int led =  LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  // Initialyze serial for Cozir sensor
  Serial1.begin(38400); // SprintIR-R-100 uses 38400 baud rate - SprintIR-WF-100 uses 9600 baud rate

  Serial.println("Testing gas analyzer START");

  pinMode(led, OUTPUT);      // set the LED pin mode
  pinMode(GPIO_LED_RED, OUTPUT);      // set the LED pin mode

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
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
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 10 seconds for connection:
  // delay(10000);
  Serial.println("BLINK");
  for(int i=0; i<15; i++){
    digitalWrite(GPIO_LED_RED, HIGH);
    delay(300);
    digitalWrite(GPIO_LED_RED, LOW);
    delay(300);
  }

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
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
 * Set the Cozir sensor to Streaming mode and reads the first full string sent
 * Return the CO2 measurement in ppm/100
 * 
 */
int readCozirStream(void){
  int iteration_limit = 0; // 
  String result = "";
  char currentChar;
  //Serial1.begin(38400); // SprintIR-R-100 uses 38400 baud rate - SprintIR-WF-100 uses 9600 baud rate

  czr.init();
  czr.setOperatingMode(CZR_STREAMING);
  // After sending set operating mode the serial hangs - serial listener must detect first Z sent.
  // delay(100);
  // Wait for 'Z' character
  //while (Serial1.available()) {
  while (iteration_limit < 2000) {
    iteration_limit++;
    currentChar = Serial1.read();
    Serial.print(currentChar);
    if (currentChar == 'Z') { // exit loop when Z char is detected
      result += currentChar;
      break;
    }
  }

  // If 'Z' character detected, read the next 6 characters
  if (currentChar == 'Z') {
    for (int i = 0; i < 6; i++) {
      while (!Serial1.available()); // Wait until data available
      // Serial.print(currentChar);
      currentChar = Serial1.read();
      result += currentChar;
    }
  }
  Serial1.end();
  int sensorValue = parseCozirStream(result);
  // Print the parsed sensor value
  Serial.print("\n-->Cozir STRING READ: ");
  Serial.println(result);
  Serial.println("-->Cozir parsed sensor value:");
  Serial.println(sensorValue);
  if(sensorValue > 0){
    return sensorValue;
  }
  else {
    return 0;
  }
}


void loop() {

  digitalWrite(GPIO_LED_GREEN, HIGH);
  Serial.println("loop START");
  //Try to read CO2 measurement
  float result_co2 = (float)readCozirStream();
  Serial.println("\n-->Cozir data (float):");
  Serial.print("CO2(%): ");
  Serial.println(result_co2/100);
  digitalWrite(GPIO_LED_GREEN, LOW);
  delay(100);

  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
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
          digitalWrite(led, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(led, LOW);                // GET /L turns the LED off
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
