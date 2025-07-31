/*
 * app-sfm3003.ino
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
 *  Created on: July 30, 2025
 *      Author: Bruno Casu
 *
 *  Version 1.0 (July 31, 2025)
 */


#include <Wire.h>
#include <PolledTimeout.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <SimpleFTPServer.h>

/* USER IMPLEMENTED START */
#define SDA_PIN 4 // D1 Mini pin function D2
#define SCL_PIN 5 // D1 Mini pin function D1
#define GPIO_SET_ACCESS_POINT 14 // D1 Mini pin function D5

// Driver implemented
#define SFM3003_CONT_MEAS_BUFF_SIZE   9
#define SFM3003_PRODUCT_ID_SIZE       4
#define SFM3003_SERIAL_NUMBER_SIZE    8
#define MEASUREMENT_INTERVAL_MS       1000

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
const char* ap_ssid = "FLOW_V1";
const char* ap_password = "flowapv1";
IPAddress local_IP(10, 0, 0, 1);
IPAddress gateway(10, 0, 0, 1);
IPAddress subnet(255, 255, 255, 0);
const int DATA_SIZE = 30;
float flowData[DATA_SIZE] = {0};
float temperatureData[DATA_SIZE] = {0};
int dataIndex = 0;
unsigned long lastMeasure = 0;
float newFlow;
float newTemp;

FSInfo fs_info;
const char* data_file_path = "/FLOW1_data.csv";
const char* csv_header_description = "FLOW(slm);TEMP(C);EX_TIME(ms)";

// FTP server access configuration
const char* user_FTP = "esp8266";
const char* pwd_FTP = "esp8266";
FtpServer ftpSrv; // Handler

const uint16_t I2C_MASTER_ADDR = 0x42;
const uint16_t I2C_SLAVE_ADDR = 0x28; // SFM3003-300-CL Address
// Commands (Cmd[15:0])
const uint16_t SFM3003_CMD_START_CONT_MEASURE_AIR = 0x3608;
const uint16_t SFM3003_CMD_START_CONT_MEASURE_02  = 0x3603;
const uint16_t SFM3003_CMD_START_CONT_MEASURE_MIX = 0x3632;
const uint16_t SFM3003_CMD_TX_CONCENTRATION       = 0xE17D;
const uint16_t SFM3003_CMD_RESET_I2C_ADDR_PTR     = 0xE000;
const uint16_t SFM3003_CMD_STOP_CONT_MEASURE      = 0x3FF9;
const uint16_t SFM3003_CMD_CONFIGURE_AVERAGING    = 0x366A;
const uint16_t SFM3003_CMD_READ_SCALE_FACTOR      = 0x3661;
const uint16_t SFM3003_CMD_GENERAL_CALL_RESET     = 0x0006;
const uint16_t SFM3003_CMD_ENTER_SLEEP_MODE       = 0x3677;
const uint16_t SFM3003_CMD_EXIT_SLEEP_MODE        = 0x0000;
const uint16_t SFM3003_CMD_READ_PROODUCT_ID       = 0xE102;
// SFM3003-300-CL product id number
const uint32_t SFM3003_300_CL_PRODUCT_ID = 0x04020110;
const uint32_t SFM3003_300_CE_PRODUCT_ID = 0x04020710;

enum Sfm3003Ctrl {
    I2C_ERROR = 0,
    I2C_SHORT_RESPONSE,
    CRC_ERROR,
    BUFFER_SIZE_ERROR,
    SFM3003_300_CL_ID,
    SFM3003_300_CE_ID,
    ID_ERROR,
    CMD_OK
};

uint8_t calculateCRC8(const uint8_t *data, size_t len);
bool checkCRC8(const uint8_t *data, size_t len, uint8_t tx_crc);
Sfm3003Ctrl sfm3003comm_send_cmd(uint16_t cmd, uint8_t *argument);
Sfm3003Ctrl sfm3003comm_read_response(uint8_t * buff, size_t len);
Sfm3003Ctrl sfm3003cmd_read_product_id(uint8_t *id, size_t id_len, uint8_t *serial_number, size_t sn_len);
Sfm3003Ctrl sfm3003cmd_check_product_id(uint8_t *id);
Sfm3003Ctrl sfm3003cmd_stop_continuous_measure(void);
Sfm3003Ctrl sfm3003cmd_start_continuous_measure_air(float *flow, float *temp, uint16_t *status_word);

bool checkCRC8(const uint8_t *data, size_t len, uint8_t tx_crc) {
    uint8_t crc = calculateCRC8(data, len);
    return (crc == tx_crc);
}

uint8_t calculateCRC8(const uint8_t *data, size_t len){
    uint8_t crc = 0xFF;          // Initial value
    uint8_t polynomial = 0x31;   // CRC-8 polynomial
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];  // XOR byte into crc
        //Serial.printf("%02x", data[i]); // Debug
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ polynomial;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

Sfm3003Ctrl sfm3003comm_send_cmd(uint16_t cmd, uint8_t *argument){
  uint8_t buff[5];
  buff[0] = uint8_t ((cmd & 0xFF00) >> 8); // cmd msb
  buff[1] = uint8_t (cmd & 0x00FF);         // cmd lsb
  Wire.beginTransmission(I2C_SLAVE_ADDR);
  Wire.write(buff[0]);
  Wire.write(buff[1]);
  if (argument){
    buff[2] = argument[0];
    buff[3] = argument[1];
    buff[4] = calculateCRC8(buff, 4);
    for(int t=0;t<3;t++){
      Wire.write(buff[2+t]);
    }
  }                   
  Wire.endTransmission();
  return CMD_OK;
}

Sfm3003Ctrl sfm3003comm_read_response(uint8_t * buff, size_t len){
  size_t k = 0, crc_count = 0;
  bool crc_check;
  Wire.requestFrom(I2C_SLAVE_ADDR, len);
  if (!Wire.available()){return I2C_ERROR;}
  while (Wire.available() && k<len) { // slave may send less than requested
    buff[k] = Wire.read();
    if(crc_count == 2){ // crc is transmitted after 2 data bytes
      crc_check = checkCRC8(&buff[k-2], 2, buff[k]);
      if(!crc_check){
        return CRC_ERROR;
      }
      crc_count = 0;
    }
    else {
      crc_count++;
    }
    k++;
  }
  if (k < len) {return I2C_SHORT_RESPONSE;}
  else {
    return CMD_OK;
  }
}

Sfm3003Ctrl sfm3003cmd_read_product_id(uint8_t *id, size_t id_len, uint8_t *serial_number, size_t sn_len){
  size_t len = 18;
  uint8_t buff[len];
  int buff_count=0;
  Sfm3003Ctrl ret_val;
  if (id_len != SFM3003_PRODUCT_ID_SIZE || sn_len != SFM3003_SERIAL_NUMBER_SIZE) {
    return BUFFER_SIZE_ERROR;
  }
  ret_val = sfm3003comm_send_cmd(SFM3003_CMD_READ_PROODUCT_ID, NULL);
  if (ret_val != CMD_OK) {return ret_val;}
  ret_val = sfm3003comm_read_response(buff, len);
  if (ret_val != CMD_OK) {return ret_val;}
  for (int i=0;i<SFM3003_PRODUCT_ID_SIZE;i++){
    buff_count++;
    if (buff_count%3 == 0 && buff_count>0){ // skip crc value
      buff_count++;
    }
    id[i] = buff[buff_count-1];
  }
  for (int n=0;n<SFM3003_SERIAL_NUMBER_SIZE;n++){
    buff_count++;
    if (buff_count%3 == 0 && buff_count>0){ // skip crc value
      buff_count++;
    }
    serial_number[n] = buff[buff_count-1];
  }
  return CMD_OK;
}

Sfm3003Ctrl sfm3003cmd_check_product_id(uint8_t *id){
    uint32_t idValue = 
        ((uint32_t)id[0] << 24) | 
        ((uint32_t)id[1] << 16) | 
        ((uint32_t)id[2] << 8)  | 
        ((uint32_t)id[3]);

    if (idValue == SFM3003_300_CL_PRODUCT_ID){
      return SFM3003_300_CL_ID;
    }
    else if (idValue == SFM3003_300_CE_PRODUCT_ID){
      return SFM3003_300_CE_ID;
    }
    else {
      return ID_ERROR;
    }
}

// When the sensor is in continuous measurement mode, 
// the sensor must be stopped before it can accept another
// command.
Sfm3003Ctrl sfm3003cmd_stop_continuous_measure(void){
  Sfm3003Ctrl ret_val = sfm3003comm_send_cmd(SFM3003_CMD_STOP_CONT_MEASURE, NULL);
  delay(10);
  return ret_val; 
}

Sfm3003Ctrl sfm3003cmd_start_continuous_measure_air(float *flow, float *temp, uint16_t *status_word){
  size_t len = 9;
  int16_t read_flow=0, read_temp=0;
  uint8_t buff[len]; // SFM3003_CONT_MEAS_BUFF_SIZE
  Sfm3003Ctrl ret_val;
  ret_val = sfm3003comm_send_cmd(SFM3003_CMD_START_CONT_MEASURE_AIR, NULL);
  delay(100);
  if (ret_val != CMD_OK) {
    // Serial.print("\n\n-->SFM3003 SEND error: "); // debug
    return ret_val;
  }
  ret_val = sfm3003comm_read_response(buff, len);
  if (ret_val != CMD_OK) {
    // Serial.print("\n\n-->SFM3003 READ error: "); // debug
    return ret_val;
  }
  read_flow = (int16_t)((buff[0] << 8) | buff[1]);
  *flow = (read_flow + 24576.0f) / 170.0f;
  read_temp = (int16_t)((buff[3] << 8) | buff[4]);
  *temp = (read_temp + 0) / 200.0f;
  *status_word = ((uint16_t)buff[6] << 8) | buff[7];
  return CMD_OK;
}


/* USER IMPLEMENTED FUNCTIONS */
void init_sfm3003_300_cl(void){
  uint8_t id[SFM3003_PRODUCT_ID_SIZE], serial_number[SFM3003_SERIAL_NUMBER_SIZE];
  Sfm3003Ctrl read_status, id_status;
  sfm3003cmd_stop_continuous_measure();
  read_status = sfm3003cmd_read_product_id(id, SFM3003_PRODUCT_ID_SIZE, serial_number, SFM3003_SERIAL_NUMBER_SIZE);
  id_status = sfm3003cmd_check_product_id(id);
  if (read_status == CMD_OK && id_status == SFM3003_300_CL_ID){
    Serial.print("\n-->SFM3003 init - found SFM3003 300 CL device");
    Serial.print("\n-->SFM3003 init - product id: ");
    for (int i=0; i<SFM3003_PRODUCT_ID_SIZE; i++){Serial.printf("%02x", id[i]);}
    Serial.print("\n-->SFM3003 init - serial number: ");
    for (int i=0; i<SFM3003_SERIAL_NUMBER_SIZE; i++){Serial.printf("%02x", serial_number[i]);}
    Serial.print("\n-->SFM3003 init - measurements: gas flow; temperature; run time");
    Serial.print("\n-->SFM3003 init - measurement units: [slm]; [°C]; [ms]) ");
  }
  else{
    Serial.print("\n-->SFM3003 init error: ");
    Serial.print(read_status);
    Serial.print(id_status);
  }
}

void data_acquisition(void){
  float flow=0, temp=0;
  uint16_t status_word=0;
  //init_sfm3003_300_cl();
  sfm3003cmd_start_continuous_measure_air(&flow, &temp, &status_word);
  delay(100);
  sfm3003cmd_stop_continuous_measure();
  Serial.print("\n-->SFM3003 readings: ");
  Serial.print(flow);
  Serial.print("; ");
  Serial.print(temp);
  Serial.print("; ");
  Serial.print(millis());
  // update globals
  newFlow = flow;
  newTemp = temp;

  if (LittleFS.exists(data_file_path)){ // File exists
    File data_file = LittleFS.open(data_file_path, "a");
    if (data_file) {
      data_file.print("\n"); // Add new line every measurement
      data_file.print(flow); // Add cycle counter value
      data_file.print(";");
      data_file.print(temp); // Add cycle counter value
      data_file.print(";");
      data_file.print(millis()); // Add cycle counter value
      data_file.flush(); // Ensure writting before returning
      data_file.close();
    }
  }
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>ESP8266 Live Data</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { text-align: center; font-family: Arial; margin: 20px; }
    .chart-container {
      position: relative;
      width: 100%;
      max-width: 800px;
      height: 400px;   /* Fixed height */
      margin: 20px auto;
    }
    .info { margin-bottom: 20px; font-size: 1.4em; font-weight: bold; }
  </style>
</head>
<body>
  <h2>Gas Flow Monitor</h2>
  <div class="info">
    Sensor1 Gas Flow: <span id="flow">0</span> SLM &nbsp;|&nbsp;
    Temperature: <span id="temp">0</span> °C
  </div>
  <div class="chart-container">
    <canvas id="chart"></canvas>
  </div>
  <script>
    const ctx = document.getElementById('chart').getContext('2d');
    let flowData = new Array(30).fill(0);
    let chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: Array.from({length: 30}, (_, i) => i - 29),
        datasets: [{
          label: 'Gas Flow (SLM)',
          data: flowData,
          borderColor: 'blue',
          backgroundColor: 'rgba(173, 216, 230, 0.4)',
          fill: true,
          tension: 0.2
        }]
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: false,  // Important for fixed height
        scales: {
          x: { title: { display: true, text: 'Time (s)' } },
          y: { beginAtZero: true }
        },
        plugins: {
          legend: { display: true, labels: { font: { size: 14 } } }
        }
      }
    });

    const ws = new WebSocket(`ws://${window.location.host}/ws`);
    ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      document.getElementById('flow').textContent = data.flow.toFixed(1);
      document.getElementById('temp').textContent = data.temperature.toFixed(1);
      flowData = data.flowData;
      chart.data.datasets[0].data = flowData;
      chart.update();
    };
  </script>
</body>
</html>
)rawliteral";


void setup() {
  Serial.begin(115200);                           // start serial for output
  pinMode(GPIO_SET_ACCESS_POINT, INPUT_PULLUP);   // set Access point pin
  Wire.begin(SDA_PIN, SCL_PIN, I2C_MASTER_ADDR);  // join i2c bus (address optional for master)
  delay(100);
  Serial.print("\n\n-->SFM3003 driver begin");
  Serial.print("\n-->SFM3003 device at I2C addr: ");
  Serial.println(I2C_SLAVE_ADDR, HEX);

  if (!LittleFS.begin()) {
    Serial.print("\n-->SFM3003 LittleFS failed");
      while(1);
  }
  // create Data files, if they do not exist
  File new_file;
  if (!LittleFS.exists(data_file_path)){ // File does not exist
    new_file = LittleFS.open(data_file_path, "w");
    if (!new_file) {
      Serial.print("\n-->SFM3003 failed creating file ");
      while(1);
    }
    else{
      new_file.print(csv_header_description);
      new_file.close();
    }
  }

  // init sfm3003 sensor
  init_sfm3003_300_cl();

  // wifi access point configuration
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("\n-->WIFI AP AT: ");
  Serial.print(WiFi.softAPIP());

  // if config pine is low, go to FTP server mode
  if (digitalRead(GPIO_SET_ACCESS_POINT) == LOW){
    digitalWrite(LED_BUILTIN, LOW); // LED on
    ftpSrv.begin(user_FTP, pwd_FTP);
    ftpSrv.setLocalIp(WiFi.softAPIP());
    while (digitalRead(GPIO_SET_ACCESS_POINT) == LOW) {
      ftpSrv.handleFTP();
      delay(10);
    }
  }

  // web server configuration
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){request->send_P(200, "text/html", index_html);});
  // ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

}


void loop() {
  if (millis() - lastMeasure >= MEASUREMENT_INTERVAL_MS) {
    lastMeasure = millis();

    data_acquisition();

    flowData[dataIndex] = newFlow;
    temperatureData[dataIndex] = newTemp;
    dataIndex = (dataIndex + 1) % DATA_SIZE;

    // JSON response
    String json = "{\"flow\":" + String(newFlow) +
                  ",\"temperature\":" + String(newTemp) +
                  ",\"flowData\":[";
    for (int i = 0; i < DATA_SIZE; i++) {
      json += String(flowData[(dataIndex + i) % DATA_SIZE]);
      if (i < DATA_SIZE - 1) json += ",";
    }
    json += "]}";

    ws.textAll(json);
  }
}
