// Wire Master Reader
// by devyte
// based on the example of the same name by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// This example code is in the public domain.


#include <Wire.h>
#include <PolledTimeout.h>

#define SDA_PIN 4 // D1 Mini pin function D2
#define SCL_PIN 5 // D1 Mini pin function D1
#define CONTINUOUS_MEASURE_BUFF_SIZE  9


const uint16_t I2C_MASTER_ADDR = 0x42;
const uint16_t I2C_SLAVE_ADDR = 0x28; //SFM3003-300-CL Address
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
const uint32_t SFM3003_PRODUCT_ID = 0x04020110;  

uint8_t calculateCRC8(const uint8_t *data, size_t len);
bool checkCRC8(const uint8_t *data, size_t len, uint8_t tx_crc);
void sfm3003_send_cmd(uint16_t cmd, uint8_t *argument);
size_t sfm3003_read_response(uint8_t * buff, size_t len);

bool checkCRC8(const uint8_t *data, size_t len, uint8_t tx_crc) {
    uint8_t crc = calculateCRC8(data, len);
    return (crc == tx_crc);
}

uint8_t calculateCRC8(const uint8_t *data, size_t len){
    uint8_t crc = 0x00;          // Initial value
    uint8_t polynomial = 0x07;   // CRC-8 polynomial

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];  // XOR byte into crc

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

void sfm3003_send_cmd(uint16_t cmd, uint8_t *argument){
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
}

size_t sfm3003_read_response(uint8_t * buff, size_t len){
  size_t k = 0, crc_count = 0;
  Wire.requestFrom(I2C_SLAVE_ADDR, len);
  
  while (Wire.available() && k<len) {// slave may send less than requested
    buff[k] = Wire.read();    
    Serial.printf("%02x", buff[k]);
    if(crc_count = 2){ // crc is transmitted after 2 data bytes
      if(!checkCRC8(&buff[k-2], 2, buff[k])){
        //Serial.print("\n-->SFM3003 ERROR read response crc check failed");
        //return 0;
      }
      crc_count = 0;
    }
    k++;
    crc_count++;
  }
  return k; // return number of bytes read
}

void setup() {
  Serial.begin(115200);                      // start serial for output
  Wire.begin(SDA_PIN, SCL_PIN, I2C_MASTER_ADDR);  // join i2c bus (address optional for master)
  delay(100);
  Serial.print("\n-->SFM3003 driver begin");
  Serial.print("\n-->SFM3003 device at I2C addr: ");
  Serial.println(I2C_SLAVE_ADDR, HEX);
}

void loop() {
  uint8_t buff[18];
  int count;
  Serial.print("\n-->SFM3003 read product id: ");
  sfm3003_send_cmd(SFM3003_CMD_READ_PROODUCT_ID, NULL);
  count = sfm3003_read_response(buff, 18);
  Serial.print("\n-->SFM3003 read count: ");
  Serial.print(count);

  delay(3000);

}
