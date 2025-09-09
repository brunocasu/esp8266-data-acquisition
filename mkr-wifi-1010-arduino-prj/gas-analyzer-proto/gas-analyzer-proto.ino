// 26-9-23 presso la cura programma installato
// 26-6-25 modifiche a Mestrino

//#include <Adafruit_SleepyDog.h>  //watchdog per l'arduino
#include "Arduino.h"
#include <FCGF.h>
#include "arduino_secrets.h"

#define GFORM_ENTRY_NUMBER  5
#define RELAY_PIN 2
#define WIFI_CONNECT_TIMEOUT_MS 10000

// Call Google Form class
GoogleForm gf;

const char* your_wifi_ssid = LAN_SSID;
const char* your_wifi_password = LAN_PASS;

String gform_id = GFORM_ID;
String gform_entries[] = {
  GFORM_ENTRY1_ID,
  GFORM_ENTRY2_ID,
  GFORM_ENTRY3_ID,
  GFORM_ENTRY4_ID,
  GFORM_ENTRY5_ID
};

const int pumpOnTime = 2000; // ms
const int pumpOffTime = 60000; // ms

const int sensorPin1 = A1;
const int sensorPin2 = A2;

float o2_reading = 0;
float co2_reading = 0;


void wifiConnectLAN(void) {
  if (WiFi.status() != WL_CONNECTED) { //  If not connected, attempt connection
    digitalWrite(LED_BUILTIN, LOW); // Led is off when trying to connect
    Serial.println("WiFi Attempting to connect...");

    gf.beginWiFi(your_wifi_ssid, your_wifi_password);
    // Optional: wait until connected or timeout
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && 
           millis() - startAttemptTime < WIFI_CONNECT_TIMEOUT_MS) { // 10s timeout
      digitalWrite(LED_BUILTIN, HIGH);
      delay(250);
      digitalWrite(LED_BUILTIN, LOW);
      delay(250);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to WiFi!");
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else {
      Serial.println("\nFailed to reconnect.");
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
  else {
    Serial.println("WiFi Connected");
    digitalWrite(LED_BUILTIN, HIGH);
  } // Turn on Led if connected
}

void sendDataGForm(void){
  String data_values[] = {
          String(o2_reading),
          String(co2_reading),
          "NA",
          "NA",
          "proto_v1.0" // Device ID
      };
  wifiConnectLAN(); // Ensures that the wifi connection is on before sending data
  gf.submit(gform_id, gform_entries, data_values, GFORM_ENTRY_NUMBER, 0);
  delay(1000);
}

void dataAcquisition(void){
  float sensorValueO2 = 0, sensorValueCO2 = 0;
  // Read LOX O2 sensor
  sensorValueO2 = analogRead(sensorPin1);
  if (sensorValueO2 > 0){
    o2_reading = sensorValueO2 * (25.95 / 1023.0);
  }
  else{
    o2_reading = 0;
  }
  // Read CO2 sensor
  sensorValueCO2 = analogRead(sensorPin2);
  if (sensorValueCO2 > 0){
    co2_reading = ((sensorValueCO2 / 7.5) - 35);
    if (co2_reading < 0){
      co2_reading = 0;
    }
  }
  else {
    co2_reading = 0;
  }
  Serial.print("\nDATA O2: ");
  Serial.println(o2_reading);
  Serial.print("\nDATA CO2: ");
  Serial.println(co2_reading);
}

void setup() {
  FCGF_DEBUG = false;
  Serial.begin(115200);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(RELAY_PIN, OUTPUT); // Relay pin
  wifiConnectLAN(); // Connect to LAN
  digitalWrite(RELAY_PIN, LOW); // Relay OFF
}

void loop() {
  digitalWrite(RELAY_PIN, HIGH); // Turn on pump
  Serial.println("\nMotor ON");
  delay(pumpOnTime);

  dataAcquisition(); // Read sensors
  sendDataGForm(); // Send data to Google Forms

  Serial.println("\nMotor OFF");
  digitalWrite(RELAY_PIN, LOW); // Turn off pump
  delay(pumpOffTime);
}
