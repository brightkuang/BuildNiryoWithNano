/*
    AS5047D.cpp
    Copyright (C) 2017 Niryo

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "AS5047D.h"
#include <SPI.h>

#define AS5047D_CMD_NOP   (0x0000)
#define AS5047D_CMD_ERRFL (0x0001)
#define AS5047D_CMD_PROG  (0x0003)
#define AS5047D_CMD_DIAAGC (0x3FFC)
#define AS5047D_CMD_MAG    (0x3FFD)
#define AS5047D_CMD_ANGLEUNC (0x3FFE)
#define AS5047D_CMD_ANGLECOM (0x3FFF)

volatile long sensor_position = 0;
volatile long last_sensor_position = 0;
volatile long sensor_position_with_rotations = 0;
volatile long motor_rotation_count = 0;

volatile long motor_position_without_offset = 0;
volatile long motor_position_steps = 0;
volatile long offset = 0;

int chipSelectPin;

static int getBit(int16_t data, int bit)
{
  return (data>>bit) & 0x01;
}

static int getParity(uint16_t data)
{
  int i,bits;
  data=data & 0x7FFF; //mask out upper bit

  //count number of bits, brute force
  bits=0;
  for(i=0; i<16; i++)
  {
    if (0 != (data & ((0x0001)<<i)))
    {
      bits++;
    }
  }
  return (bits & 0x01); //return 1 if odd
}

/*
 * Ask for position register
 */
void init_position_sensor()
{
	pinMode(PIN_AS5047D_CS,OUTPUT);
  digitalWrite(PIN_AS5047D_CS,LOW);
  pinMode(PIN_AS5047D_PWR,OUTPUT);
  digitalWrite(PIN_AS5047D_PWR,HIGH);
  pinMode(PIN_MOSI,OUTPUT);
  digitalWrite(PIN_MOSI,LOW);
  pinMode(PIN_SCK,OUTPUT);
  digitalWrite(PIN_SCK,LOW);
  pinMode(PIN_MISO,INPUT);
  
	digitalWrite(PIN_AS5047D_PWR,HIGH);
  digitalWrite(PIN_AS5047D_CS,LOW); //pull CS LOW by default (chip powered off)
  digitalWrite(PIN_MOSI,LOW);
  digitalWrite(PIN_SCK,LOW);
  digitalWrite(PIN_MISO,LOW);
  pinMode(PIN_MISO,OUTPUT);
  delay(1000);

  digitalWrite(PIN_AS5047D_CS,HIGH); //pull CS high
  digitalWrite(PIN_AS5047D_PWR,LOW);

  pinMode(PIN_MISO,INPUT);

  SPISettings settingsA(5000000, MSBFIRST, SPI_MODE1);             ///400000, MSBFIRST, SPI_MODE1);
  chipSelectPin=PIN_AS5047D_CS;

  SerialUSB.print("chipSelectPin is ");
  SerialUSB.println(chipSelectPin);
  pinMode(chipSelectPin,OUTPUT);
  digitalWrite(chipSelectPin,HIGH); //pull CS high by default
  delay(1);
  SPI.begin();    //AS5047D SPI uses mode=1 (CPOL=0, CPHA=1)
  SerialUSB.println("Begin AS5047D...");

  SPI.beginTransaction(settingsA);
  SPI.transfer16(AS5047D_CMD_NOP);
  delay(10);

  //wait for the LF bit to be set
  uint16_t data=0,t0=2000;
  while (getBit(data,8)==0)
  {
    delay(1);
    t0--;
    if (t0==0)
    {
      SerialUSB.println("LF bit not set");
    }
    SerialUSB.print("AS5047D_CMD_DIAAGC:");
    SerialUSB.println(data);
    data=readAddress(AS5047D_CMD_DIAAGC);
  }
}

//read the encoders 
int16_t readAddress(uint16_t addr)
{
  uint16_t data;
  
  //make sure it is a read by setting bit 14
  addr=addr | 0x4000;

  //add the parity to the command
  if (1 == getParity(addr))
  {
    addr=(addr & 0x7FFF) | 0x8000; //add parity bit to make command even number of bits
  }

  digitalWrite(chipSelectPin, LOW);
  delayMicroseconds(1);
  //clock out the address to read
  SPI.transfer16(addr);
  digitalWrite(chipSelectPin, HIGH);
  delayMicroseconds(1);
  digitalWrite(chipSelectPin, LOW);
  //clock out zeros to read in the data from address
  data=SPI.transfer16(0x00);

  digitalWrite(chipSelectPin, HIGH);

  if (data & (1<<14))
  {
    //if bit 14 is set then we have an error
    SerialUSB.print("read command failed"); 
    SerialUSB.println(addr); 
    return -1;
  }

  if (data>>15 != getParity(data))
  {
    //parity did not match
    SerialUSB.print("read command parity error 0x%04X ");
    SerialUSB.println(addr);
    return -2;
  }

  data=data & 0x3FFF; //mask off the error and parity bits

  return data;
}

/*
 * Read sensor position register
 */
int read_encoder()
{
  int angle;
  
  angle=((uint32_t)readAddress(AS5047D_CMD_ANGLEUNC)) >> 3;

  return angle;  
}

void update_current_position(int microsteps) 
{
  // read from encoder
  sensor_position = read_encoder() * 2;

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
