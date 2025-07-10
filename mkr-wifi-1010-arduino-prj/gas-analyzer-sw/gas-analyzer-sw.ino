/*
 * gas-analyzer-sw.ino
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
 *  Created on: July 10, 2025
 *      Author: Bruno Casu
 *
 *  Version 1.0 (July 10, 2025)
 */

// SW for Arduino MKR WiFi 1010 board
 
//#include <ESP8266WiFi.h>
#include <Hash.h>
//#include <ESPAsyncTCP.h>
//#include <ESPAsyncWebServer.h>
#include <PolledTimeout.h>
#include <LittleFS.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <SimpleFTPServer.h> // Replacing <ESP8266FtpServer.h>
#include <cozir.h>
#include <sht30.h>
//#include <LOLIN_HP303B.h>
#include <math.h>
#include <cozir_stream.h>
#include <FCGF.h>

#define SERIAL_SPEED  115200
#define APP_VERSION "gas-analyzer-sw_v1.0"

// Defines for the data aquisition system
#define SHT30_I2C_ADDR_PIN_HIGH 0x45  // Jumper NOT connected
#define SHT30_I2C_ADDR_PIN_LOW 0x44  // Jumper connected

// #define GPIO_SET_ACCESS_POINT 14 // On Wemos D1 Mini - Pin number 14 (GPIO14)
#define GPIO_RED_LED  7
#define GPIO_GREEN_LED  6
#define GPIO_COZIR_TX  1
#define GPIO_COZIR_RX  1
#define GPIO_LOX_TX  1
#define GPIO_LOX_RX  1

// Maximum attempts to read LOX-O2 sensor
#define LOX_MAX_READ_ATTEMPTS 10
// Remove when deploying in production environment
#define DEBUG_MODE

#define CYCLE_COUNTER_RST 10000000
unsigned long cycle_counter = 0;

// Temperature and humidity sensor
SHT30 sht30_1_handler(SHT30_I2C_ADDR_PIN_HIGH); // Addr 0x45
SHT30 sht30_2_handler(SHT30_I2C_ADDR_PIN_LOW); // Addr 0x44

// WiFi Access Point configuration
IPAddress local_IP(100,100,100,1); // FTP server address
IPAddress gateway(100,100,100,1);
IPAddress subnet_mask(255,255,255,0);
const char* ssid_AP = "GAS-ANALYZER-MKR1010"; // No password set

// FTP server access configuration
const char* user_FTP = "mkr1010";
const char* pwd_FTP = "mkr1010";
FtpServer ftpSrv; // Handler

// LittleFs info
FSInfo fs_info;

// Cozir CO2 sensor 
//SoftwareSerial swsCozir(15, 14);  // RX, TX, optional inverse logic 13, 12
//SoftwareSerial swsCozir(13, 12);
//COZIR czr(&swsCozir);

// LOX-O2 sensor
//SoftwareSerial swsLox(14, 15); // Port 14 (RX) goes into LOX-02 Port 3. Port 15 (TX) goes into LOX-02 Port 4.

const char* data_file_path = "/ga01_data.csv";
// Added at the creation of the Data file
const char* csv_header_description = "CTR;HP303B_T(C);HP303B_P(Pa);SHT30_45_T(C);SHT30_45_RH(%);SHT30_44_T(C);SHT30_44_RH(%);PPMV;DP(C);ABS_HUM(g/m3);CO2(%);EX_t(ms)"; 

// HTTP server

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Sensor values, updated in dataAcquisition()
// variables made global so they can be accessed by the web server callback
float read_o2=0; 
float read_co2=0;
double hp_T=0;
int32_t hp_P=0;
float hp_P_mbar=0;
// int32_t temp; // not used (integer value of temeprature reading)
float sht_T=0;
float sht_RH=0;
// float arr[5]={0}; // return values of the calculatePPMV func: [0]=Pws, [1]=Pw, [2]=ppmv, [3]=dew_point, [4]=abs_hum
float dew_point=0; // Deg Celsius
float abs_hum=0; // g/m3

unsigned long previousMillis = 0;

// HTTP page
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html {
      font-family: Arial;
      display: inline-block;
      margin: 0px auto;
      text-align: center;
    }
    h2 { font-size: 2.0rem; }
    p { font-size: 2.0rem; }
    .units { font-size: 1.2rem; }
    .param-labels {
      font-size: 1.2rem;
      vertical-align: middle;
      padding-bottom: 15px;
    }
  </style>
</head>
<body>
  <h2>ESP8266 Server</h2>
  <p>
    <span class="param-labels">SHT30 Temperature</span> 
    <span id="temperature">--</span>
    <sup class="units">&deg;C</sup>
  </p>
  <p>
    <span class="param-labels">SHT30 RHumidity</span>
    <span id="humidity">--</span>
    <sup class="units">%</sup>
  </p>
  <p>
    <span class="param-labels">HP303 Temperature</span> 
    <span id="temperature2">--</span>
    <sup class="units">&deg;C</sup>
  </p>
  <p>
    <span class="param-labels">HP303 Pressure</span>
    <span id="pressure">--</span>
    <sup class="units">mbar</sup>
  </p>
  <p>
    <span class="param-labels">Dew Point</span> 
    <span id="dew_point">--</span>
    <sup class="units">&deg;C</sup>
  </p>
  <p>
    <span class="param-labels">Abs Humidity</span>
    <span id="abs_humidity">--</span>
    <sup class="units">g/m3</sup>
  </p>
    <p>
    <span class="param-labels">CO2</span>
    <span id="co2">--</span>
    <sup class="units">%</sup>
  </p>
    <p>
    <span class="param-labels">O2</span>
    <span id="o2">--</span>
    <sup class="units">%</sup>
  </p>
</body>
<script>
  function updateSensorValues() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
      if (this.readyState == 4 && this.status == 200) {
        var data = JSON.parse(this.responseText); // Parse the JSON response
        document.getElementById("temperature").innerHTML = data.temperature;
        document.getElementById("humidity").innerHTML = data.humidity;
        document.getElementById("temperature2").innerHTML = data.temperature2;
        document.getElementById("pressure").innerHTML = data.pressure;
        document.getElementById("dew_point").innerHTML = data.dew_point;
        document.getElementById("abs_humidity").innerHTML = data.abs_humidity;
        document.getElementById("co2").innerHTML = data.co2;
        document.getElementById("o2").innerHTML = data.o2;
      }
    };
    xhttp.open("GET", "/getSensorValues", true); // Fetch data from the server
    xhttp.send();
  }
  // Update sensor values every second
  setInterval(updateSensorValues, 1000);
</script>
</html>)rawliteral";


/** PF definitions **/

/**
 * Error handler function
 */
void errorHandler(){
  for(;;){
    digitalWrite(LED_BUILTIN, LOW); // LED on
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH); // LED off
    delay(100);
  }
}

/**
 * Serial print the device configuration and file system info
 */
void printDeviceInfo(){
  LittleFS.info(fs_info);
  Serial.begin(SERIAL_SPEED);
  delay(50);
  Serial.print("\n\n-->SW VERSION: ");
  Serial.print("\n\n-->DEMO WEB SERVER APP");
  Serial.println(APP_VERSION);
  Serial.print(F("Cycle counter = "));
  Serial.println(cycle_counter);
  Serial.print("\n-->REFRESH TIME (ms): ");
  Serial.println(REFRESH_TIME_MS);
  // FS info
  Serial.print("-->LittleFS Mount OK\nTotal FS Size (kB):");
  Serial.println(fs_info.totalBytes*0.001);
  Serial.print("Used (kB): ");
  Serial.println(fs_info.usedBytes*0.001);
  // FTP-AP info
  Serial.print("\n-->AP Mode - WiFi SSID (NO PASSWORD): ");
  Serial.println(ssid_AP);
  Serial.print("\n-->Server IP Addr: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Plain FTP Connection (insecure)\nFTP_USR: ");
  Serial.println(user_FTP);
  Serial.print("FTP_PWD: ");
  Serial.println(pwd_FTP);
  Serial.flush();
}

/**
 * Configures the ESP8266 as WiFi Access Point
 */
void setAccessPoint() {
  // Configure WiFi Access point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid_AP); // No password for connecting
  WiFi.softAPConfig(local_IP, gateway, subnet_mask);
  printDeviceInfo();
}

void setFtpServer() {
  // Start FTP server
  ftpSrv.begin(user_FTP, pwd_FTP);
  ftpSrv.setLocalIp(WiFi.softAPIP()); // Must setLocalIP as the same as the one in softAPConfig()
  while (digitalRead(GPIO_SET_ACCESS_POINT) == LOW) {
    ftpSrv.handleFTP();
    delay(10);
  }
}

/**
 * Create data files for each sensor
 */
void createDataFiles(){
  File new_file;
  // Create Data files, if they do not exist
  if (!LittleFS.exists(data_file_path)){ // File does not exist
    new_file = LittleFS.open(data_file_path, "w");
    if (!new_file) {
      errorHandler(); // Failed creating file
    }
    else{
      new_file.print(csv_header_description); // Write CSV header
      new_file.close();
    }
  }
}

/**
 * Calculate the water vapor content, absolute humidity and dew point based on T, P and RH measurements
 */
void calculatePPMV(float *arr, float T, int32_t P_Pa, float RH){
    float P = (float) P_Pa/100; // Measured Pressure in hPa (mbar)
    float Pws=0; // Water vapor saturation pressure (hPa)
    float Pw=0; // Water vapor pressure (hPa)
    float ppmv=0; // Water vapor content Volume/Volume (ppmv)(wet)
    //float dew_point=0; // Deg Celsius
    //float abs_hum=0; // g/m3
    float A=6.116441, m=7.591386, Tn=240.7263, C=2.16679; // Constants from Vaisala
    if (P!=0 && RH!=0){
      Pws = (float) A * pow(10, ((m * T) / (Tn + T))); // Equations from Vaisala
      //Pws = (float) pC * 6.1121 * exp((pA * T) / (pB + T));
      Pw = (float) Pws*(RH/100);
      ppmv = (float) 1000000 * (Pw/P);
      float div = (float) log10(Pw/A);
      dew_point = (float) Tn / ((m/div)-1);
      abs_hum = (float) C * ((Pw*100)/(T+273.15));
      arr[0] = Pws;
      arr[1] = Pw;
      arr[2] = ppmv;
      arr[3] = dew_point;
      arr[4] = abs_hum;
    }
#ifdef DEBUG_MODE
    Serial.println("\n-->Sensor data:");
    Serial.print("HP303B_T(C): ");
    Serial.println(T);
    Serial.print("HP303B_P(hPa): ");
    Serial.println(P);
    Serial.print("HP303B_P(Pa): ");
    Serial.println(P_Pa);
    Serial.print("SHT30_RH(%): ");
    Serial.println(RH);
    Serial.print("Pws(hPa): ");
    Serial.println(Pws, 4);
    Serial.print("Pw(hPa): ");
    Serial.println(Pw, 4);
    Serial.print("PPMV: ");
    Serial.println(ppmv, 0);
    Serial.print("Dew point(C): ");
    Serial.println(dew_point);
    Serial.print("Abs Humidity (g/m3): ");
    Serial.println(abs_hum);
#endif // DEBUG_MODE
}

/**
 * Read and save sensor data on data file
 */
void dataAcquisition(){
  float result_co2=0;
  float result_o2=0;
  //double hp_T=0;
  int32_t temp; // not used (integer value of temeprature reading)
  //int32_t hp_P=0;
  //float sht_RH=0;
  float arr[5]={0}; // return values of the calculatePPMV func: [0]=Pws, [1]=Pw, [2]=ppmn, [3]=dew_point, [4]=abs_hum
  if (LittleFS.exists(data_file_path)){ // File exists
    File data_file = LittleFS.open(data_file_path, "a");
    if (!data_file) {
      return;
    }
    data_file.print("\n"); // Add new line every measurement
    data_file.print(cycle_counter); // Add cycle counter value
    data_file.print(";");
    // Try to read HP303B
    hp303b_handler.begin();
    if(hp303b_handler.measureTempOnce(temp, HP303B_OVERSAMPLING_RATE) == 0){ // Temperature reading OK
      hp_T = hp303b_handler.returnDoublePrecisionTemp();
      data_file.print(hp_T, 2);
    }
    data_file.print(";");
    if(hp303b_handler.measurePressureOnce(hp_P, HP303B_OVERSAMPLING_RATE) == 0){ // Pressure reading OK
      hp_P_mbar = hp_P/100;
      data_file.print(hp_P);
    }
    data_file.print(";");
    // Try to read SHT30 addr 45
    if(sht30_1_handler.read_single_shot() == SHT30_READ_OK){
      sht_RH = (float) sht30_1_handler.humidity; // To calculate the PPMV levels, the RH from SHT30 Addr 45 is used by default
      sht_T = (float) sht30_1_handler.cTemp;
      data_file.print(sht30_1_handler.cTemp, 2);
      data_file.print(";");
      data_file.print(sht30_1_handler.humidity, 2);
      data_file.print(";");
    }
    else{data_file.print(";;");} // Empty data
    // Try to read SHT30 addr 44
    if(sht30_2_handler.read_single_shot() == SHT30_READ_OK){
      data_file.print(sht30_2_handler.cTemp, 2);
      data_file.print(";");
      data_file.print(sht30_2_handler.humidity, 2);
      data_file.print(";");
    }
    else{data_file.print(";;");} // Empty data
    // Calculate and write PPMV and DP values
    calculatePPMV(arr, hp_T, hp_P, sht_RH);
    if (arr[0]!=0 && arr[1]!=0){
      data_file.print(arr[2], 0); // ppmv
      data_file.print(";");
      data_file.print(arr[3], 2); // dew_point
      data_file.print(";");
      data_file.print(arr[4], 2); // abs_hum
      data_file.print(";");
    }
    else{data_file.print(";;;");} // Empty data

    //Try to read CO2 measurement
    result_co2 = (float)readCozirStream();
    if(result_co2>0){
      read_co2 = result_co2/100; // update global only if return valus is != 0
      data_file.print(read_co2/100, 2);
      data_file.print(";");
#ifdef DEBUG_MODE
      Serial.println("\n-->Cozir data (float):");
      Serial.print("CO2(%): ");
      Serial.println(read_co2/100);
#endif // DEBUG_MODE  
    }

    data_file.print(";");
    // Try to read O2 measurement
    //swsLox.begin(9600);
    result_o2 = readLoxStream();
    if (result_o2 > 0){
      read_o2 = result_o2;
      data_file.print(read_o2, 2);
    }
    
    data_file.print(";");
    data_file.print(millis());
    data_file.flush(); // Ensure writting before returning
    data_file.close();
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
  swsCozir.begin(9600);
  czr.init();
  czr.setOperatingMode(CZR_STREAMING);
  // After sending set operating mode the serial hangs - serial listener must detect first Z sent.
  // delay(100);
  // Wait for 'Z' character
  //while (swsCozir.available()) {
  while (iteration_limit < 2000) {
    iteration_limit++;
    currentChar = swsCozir.read();
    Serial.print(currentChar);
    if (currentChar == 'Z') { // exit loop when Z char is detected
      result += currentChar;
      break;
    }
  }

  // If 'Z' character detected, read the next 6 characters
  if (currentChar == 'Z') {
    for (int i = 0; i < 6; i++) {
      while (!swsCozir.available()); // Wait until data available
      // Serial.print(currentChar);
      currentChar = swsCozir.read();
      result += currentChar;
    }
  }
  swsCozir.end();
#ifdef DEBUG_MODE
  Serial.print("\n-->Cozir STRING READ: ");
  Serial.println(result);
#endif // DEBUG_MODE  
  int sensorValue = parseCozirStream(result);
  // Print the parsed sensor value
  if(sensorValue > 0){
    return sensorValue;
  }
  else {
    return 0;
  }
}


/**
 * This function parses the inut string to retrieve the O2 measurement
 * The LOX-O2 sensor on Streaming mode sends a string with multiple information
 * When retrieving the characters after the "%" character is sent, the info is in
 * the format "% 020.04" (example). To retrieve the float value of the O2%, the
 * implemented function parses the string and checks if the format is correct.
 * The function returns the Oxygen concentration level in percentage.
 */
float parseLoxStream(String input) {
  // Check if the input string starts with "% " and has at least 7 characters (e.g., "% 020.04")
  if (input.length() >= 7 && input[0] == '%' && input[1] == ' ') {
    // Try to extract the number part
    String numberPart = input.substring(2); // Get the part after "% "
    
    // Check if the number part is a valid float
    bool isValidNumber = true;
    bool decimalPointFound = false;
    int digitsAfterDecimal = 0;
    
    for (int i = 0; i < numberPart.length(); i++) {
      char c = numberPart[i];
      
      // Check for valid number characters
      if (isDigit(c)) {
        if (decimalPointFound) {
            digitsAfterDecimal++;
        }
      } else if (c == '.' && !decimalPointFound) {
        // Allow only one decimal point
        decimalPointFound = true;
      } else {
        isValidNumber = false; // Invalid character found
        break;
      }
    }
    // If valid number, parse and return it
    if (isValidNumber && digitsAfterDecimal <= 2) {
      return numberPart.toFloat();
    }
  }
  // If the format is incorrect, return 0
  return 0;
}


/**
 * Set the LOX-O2 sensor to Streaming mode and reads the O2% data from the stream
 * Return the O2% measurement (2 decimals)
 * 
 */
float readLoxStream(void){
  String result = "";
  char currentChar;
  int r=0;
  float sensorValue = 0;
  swsLox.begin(9600);
  //sws.write("M 1\r\n");
  // Wait for '%' character
  while(r<LOX_MAX_READ_ATTEMPTS && sensorValue == 0){
    while (swsLox.available()) {
      currentChar = swsLox.read();
      if (currentChar == '%') {
        result += currentChar;
        for (int i = 0; i < 7; i++) { // If '%' character detected, read the next 7 characters
          while (!swsLox.available()); // Wait until data available
          currentChar = swsLox.read();
          result += currentChar;
        }
        break;
      }
    }
    sensorValue = parseLoxStream(result);
    r++;
    result = "";
    delay(100);
  }
#ifdef DEBUG_MODE
  Serial.print("\n-->LOX-O2 READ O2(%): ");
  Serial.println(sensorValue);
#endif // DEBUG_MODE  
  return sensorValue;
}

// Serve the HTML page
void handleRoot(AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", index_html);
}


// Serve sensor values as JSON
void handleSensorValues(AsyncWebServerRequest *request) {
  String json = "{";
  json += "\"temperature\":" + String(sht_T) + ",";
  json += "\"humidity\":" + String(sht_RH) + ",";
  json += "\"temperature2\":" + String(hp_T) + ",";
  json += "\"pressure\":" + String(hp_P_mbar) + ",";
  json += "\"dew_point\":" + String(dew_point) + ",";
  json += "\"abs_humidity\":" + String(abs_hum) + ",";
  json += "\"co2\":" + String(read_co2) + ",";
  json += "\"o2\":" + String(read_o2);
  json += "}";
  request->send(200, "application/json", json);
}


void setup() {
  system_rtc_mem_read(64, &cycle_counter, 4); // Copy RTC memory value in current cycle counter
  if (cycle_counter > CYCLE_COUNTER_RST){cycle_counter = 0;} // This prevents errors in first boot if done by GPIO reset (RTC memeory gets wrong value)
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED off
  pinMode(GPIO_SET_ACCESS_POINT, INPUT_PULLUP); // Set Access point pin
  Wire.begin(); // I2C
  //WiFi.mode(WIFI_OFF); // Must turn the modem off; using disconnect won't work
  //WiFi.forceSleepBegin();
  // Mount file system
  if (!LittleFS.begin()) {
    errorHandler(); // LittleFS mount failed
  }
  createDataFiles();

  setAccessPoint();

  // Route for root / web page
  server.on("/", HTTP_GET, handleRoot);
  // Route for updating sensor values
  server.on("/getSensorValues", HTTP_GET, handleSensorValues);

  // Start server
  server.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  
#ifdef DEBUG_MODE
  printDeviceInfo();
#endif // DEBUG_MODE  

}


void loop() { 
  unsigned long currentMillis = millis();

  
  
  //if (currentMillis - previousMillis >= REFRESH_TIME_MS) {
  //  // Sensor reading interval
  //  previousMillis = currentMillis;
  //  dataAcquisition(); // update global variables
  //}

  //if (digitalRead(GPIO_SET_ACCESS_POINT) == LOW){ // Change to FTP mode
  //  digitalWrite(LED_BUILTIN, LOW); // LED on
    //cycle_counter = 0;
    //system_rtc_mem_write(64, &cycle_counter, 4); // Reset the cycle counter
  //  setFtpServer();
  //}

}
