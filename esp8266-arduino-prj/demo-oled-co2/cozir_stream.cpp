/*
 * cozir_stream.cpp
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
 *  Created on: July 16, 2024
 *      Author: Bruno Casu
 */


#include "cozir_stream.h"

/**
 *
 */
CZR_STREAM::CZR_STREAM(int rx_pin, int tx_pin){
  _rx = rx_pin;
  _tx = tx_pin;
}

/**
 * Return the CO2 measurement in percentage (configured for 0-100% CO2)
 *
 */
float CZR_STREAM::read_co2(void){
  float co2;
  // Init software serial
  SoftwareSerial sws(_rx, _tx);  // RX, TX, optional inverse logic
  // Init Cozir sensor
  COZIR czr(&sws);
  sws.begin(9600);
  czr.init();
  // Read CO2 measurement
  co2 = read_cozir_stream(czr);
  if (co2>0){
    return co2/100;
  else
    return 0;
}

/**
 * Set the Cozir sensor to Streaming mode
 */
int CZR_STREAM::read_cozir_stream(COZIR czr){
  czr.setOperatingMode(CZR_STREAMING); // Set mode to stream data
  String czr_string = "";
  char current_char;
  // Wait for 'Z' character
  while (sws.available()) {
    current_char = sws.read();
    if (current_char == 'Z') {
      czr_string += current_char;
      break;
    }
  }
  // If 'Z' character detected, read the next 6 characters
  if (current_char == 'Z') {
    for (int i = 0; i < 6; i++) {
      while (!sws.available()); // Wait until data available
      current_char = sws.read();
      czr_string += current_char;
    }
  }
  // Serial.println(czr_string);
  int parsed_co2_reading = parse_stream_string(czr_string);
  if(parsed_co2_reading > 0){
    return parsed_co2_reading;
  }
  else {
    return -1;
  }
}

/**
 * The Cozir sensor stream characters on the format "Z 0000"
 * The parser returns the numerical value of the string
 */
int CZR_STREAM::parse_stream_string(String input) {
  // Find the position of 'Z' in the input string
  int z_pos = input.indexOf('Z');

  // If 'Z' is found
  if (z_pos != -1) {
    // Extract the substring starting from 'Z' position
    String number_string = input.substring(z_pos + 2); // Skip 'Z ' and the following space to get the number part
    number_string.trim();

    // Convert the string to an integer
    int val = number_string.toInt();

    // Return the parsed integer value (CO2 reading)
    return val;
  }
  else {
    return -1;
  }
}
