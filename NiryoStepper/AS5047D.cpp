/**********************************************************************
 *      Author: tstern
	Copyright (C) 2018  MisfitTech,  All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    Written by Trampas Stern for MisfitTech.

    Misfit Tech invests time and resources providing this open source code,
    please support MisfitTech and open-source hardware by purchasing
	products from MisfitTech, www.misifittech.net!

 *********************************************************************/
#include <Arduino.h>
#include "as5047d.h"
#include "SPI.h"
#include <stdio.h>
#include "wiring_private.h" // pinPeripheral() function

#define AS5047D_CMD_NOP   (0x0000)
#define AS5047D_CMD_ERRFL (0x0001)
#define AS5047D_CMD_PROG  (0x0003)
#define AS5047D_CMD_DIAAGC (0x3FFC)
#define AS5047D_CMD_MAG    (0x3FFD)
#define AS5047D_CMD_ANGLEUNC (0x3FFE)
#define AS5047D_CMD_ANGLECOM (0x3FFF)

#define RD  0x40    // bit 14 "1" is Read + parity even

#pragma GCC push_options
#pragma GCC optimize ("-Ofast")

static int getBit(int16_t data, int bit)
{
	return (data>>bit) & 0x01;
}

void AS5047D_setup()
{
	pinMode(AS5047D_PIN_CS,OUTPUT);
 
	digitalWrite(AS5047D_PIN_CS,HIGH);                     //pull CS high by default
	delay(1);
	SPI.begin();                                          //AS5047D SPI uses mode=1 (CPOL=0, CPHA=1)
  SPI.setDataMode(SPI_MODE1);                           // properties chip
  SPI.setBitOrder(MSBFIRST);                            //properties chip 
  
  AS5047D_Write( AS5047D_PIN_CS ,SETTINGS1, 0x0004);
  AS5047D_Write( AS5047D_PIN_CS ,SETTINGS2, 0x0000);
  
}

/*
 * Read sensor position register
 */
int read_encoder()
{
  int angle;
  
  uint16_t ANGLEUNC = AS5047D_Read( AS5047D_select_pin, ANGLECOM) & 0x3FFF;
  
  angle = ANGLEUNC;

  return angle;  
}

void update_current_position(int microsteps) 
{
  // read from encoder
  sensor_position = read_encoder();

  // check if motor did one rotation
  if (sensor_position - last_sensor_position < - AS5047D_CPR_HALF) {
    ++motor_rotation_count;
  }
  else if (sensor_position - last_sensor_position > AS5047D_CPR_HALF) {
    --motor_rotation_count;
  }

  // get total sensor position
  sensor_position_with_rotations = sensor_position + AS5047D_CPR * motor_rotation_count;
  last_sensor_position = sensor_position;

  // translate sensor position to motor micro steps
  motor_position_without_offset = (sensor_position_with_rotations * microsteps * STEPPER_CPR) / AS5047D_CPR;
  motor_position_steps =  motor_position_without_offset - offset;
}

// ************************Write to AS5047D **************************
void AS5047D_Write( int SSPin, int address, int value)
{
  // take the SS pin low to select the chip:
  digitalWrite(SSPin, LOW);
  
  //  send in the address via SPI:
  
  byte v_l = address & 0x00FF;
  byte v_h = (unsigned int)(address & 0x3F00) >> 8;
  
  if (parity(address & 0x3F) == 1) v_h = v_h | 0x80; // set parity bit

  SPI.transfer(v_h);
  SPI.transfer(v_l);
  
  digitalWrite(SSPin, HIGH);
  
  delay(2);
  
  digitalWrite(SSPin, LOW);
  
  //  send value via SPI:
  
  v_l = value & 0x00FF;
  v_h = (unsigned int)(value & 0x3F00) >> 8;
  
  if (parity(value & 0x3F) == 1) v_h = v_h | 0x80; // set parity bit
  
  SPI.transfer(v_h);
  SPI.transfer(v_l);
  
  // take the SS pin high to de-select the chip:
  digitalWrite(SSPin, HIGH);
}

//*******************Read from AS5047D ********************************
unsigned int AS5047D_Read( int SSPin, unsigned int address)
{
  unsigned int result = 0;   // result to return
  
  byte res_h = 0;
  byte res_l = 0;
  
  // take the SS pin low to select the chip:
  digitalWrite(SSPin, LOW);
  
  //  send in the address and value via SPI:
  byte v_l = address & 0x00FF;
  byte v_h = (unsigned int)(address & 0x3F00) >> 8;
  
  if (parity(address | (RD << 8)) == 1) v_h = v_h | 0x80; // set parity bit
  
  v_h = v_h | RD; // its  a read command
  
  res_h = SPI.transfer(v_h);
  res_l = SPI.transfer(v_l);
  
  digitalWrite(SSPin, HIGH);
  
  delay(2);
  
  digitalWrite(SSPin, LOW);
  
  //if (parity(0x00 | (RD <<8))==1) res_h = res_h | 0x80;  // set parity bit
  //res_h = res_h | RD;
  
  res_h = (SPI.transfer(0x00));
  res_l = SPI.transfer(0x00);
  
  res_h = res_h & 0x3F;  // filter bits outside data
  
  digitalWrite(SSPin, HIGH);
  
  return (result = (res_h << 8) | res_l);
}

//*******************check parity ******************************************
int parity(unsigned int x) {
  int parity = 0;
  while (x > 0) {
    parity = (parity + (x & 1)) % 2;
    x >>= 1;
  }
  return (parity);
}

#pragma GCC pop_options
