/*
 * cozir_continous.h
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

#ifndef COZIR_STREAM_H
#define COZIR_STREAM_H

#include "Arduino.h"
#include "cozir.h"
#include "SoftwareSerial.h"
#include <Wire.h>

class CZR_STREAM{
  public:
    CZR_STREAM(int rx_pin, int tx_pin);
    float read_co2(void);
  private:
    int   read_cozir_stream(COZIR czr);
    int   parse_stream_string(String input);
    int   _rx;
    int   _tx;
}

#endif
