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
#include "SPI.h"

volatile long sensor_position = 0;
volatile long last_sensor_position = 0;
volatile long sensor_position_with_rotations = 0;
volatile long motor_rotation_count = 0;

volatile long motor_position_without_offset = 0;
volatile long motor_position_steps = 0;
volatile long offset = 0;

#define AS5047D_CMD_NOP   (0x0000)
#define AS5047D_CMD_ERRFL (0x0001)
#define AS5047D_CMD_PROG  (0x0003)
#define AS5047D_CMD_DIAAGC (0x3FFC)
#define AS5047D_CMD_MAG    (0x3FFD)
#define AS5047D_CMD_ANGLEUNC (0x3FFE)
#define AS5047D_CMD_ANGLECOM (0x3FFF)

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
boolean AS5047D::init_position_sensor()
{
  digitalWrite(PIN_AS5047D_CS,LOW); //pull CS LOW by default (chip powered off)
  digitalWrite(PIN_MOSI,LOW);
  digitalWrite(PIN_SCK,LOW);
  digitalWrite(PIN_MISO,LOW);
  pinMode(PIN_MISO,OUTPUT);
  delay(1000);

  digitalWrite(PIN_AS5047D_CS,HIGH); //pull CS high
  pinMode(PIN_MISO,INPUT);

  error=false;
  SPISettings settingsA(5000000, MSBFIRST, SPI_MODE1);             ///400000, MSBFIRST, SPI_MODE1);

  pinMode(PIN_AS5047D_CS,OUTPUT);
  digitalWrite(PIN_AS5047D_CS,HIGH); //pull CS high by default
  delay(1);

  SPI.begin();    //AS5047D SPI uses mode=1 (CPOL=0, CPHA=1)
  Serial.println("Begin AS5047D...");

  SPI.beginTransaction(settingsA);
  SPI.transfer16(0x0000);
  delay(10);

  //wait for the LF bit to be set
  uint16_t data=0,t0=2000;
  while (getBit(data,8)==0)
  {
    delay(1);
    t0--;
    if (t0==0)
    {
      Serial.println("LF bit not set");
      error=true;
      return false;
    }
    Serial.print("data is ");
    Serial.println(data);
    data=readAddress(AS5047D_CMD_DIAAGC);
  }
  return true;
}

//read the encoders 
int16_t AS5047D::readAddress(uint16_t addr)
{
  uint16_t data;
  error=false;
  //make sure it is a read by setting bit 14
  addr=addr | 0x4000;

  //add the parity to the command
  if (1 == getParity(addr))
  {
    addr=(addr & 0x7FFF) | 0x8000; //add parity bit to make command even number of bits
  }

  digitalWrite(PIN_AS5047D_CS, LOW);
  delayMicroseconds(1);
  //clock out the address to read
  SPI.transfer16(addr);
  digitalWrite(PIN_AS5047D_CS, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_AS5047D_CS, LOW);
  //clock out zeros to read in the data from address
  data=SPI.transfer16(0x00);

  digitalWrite(PIN_AS5047D_CS, HIGH);

  if (data & (1<<14))
  {
    //if bit 14 is set then we have an error
    Serial.print("read command failed: ");
    Serial.println(addr);
    error=true;
    return -1;
  }

  if (data>>15 != getParity(data))
  {
    //parity did not match
    Serial.print("read command parity error: ");
    Serial.println(addr);
    error=true;
    return -2;
  }

  data=data & 0x3FFF; //mask off the error and parity bits

  return data;
}

//read the encoders 
int16_t AS5047D::readEncoderAngle(void)
{
  return readAddress(AS5047D_CMD_ANGLECOM);
}

/*
 * Read sensor position register
 */
int AS5047D::read_encoder()
{
  int angle;
  
  Wire.requestFrom(AS5047D_ADDRESS, 2);
    
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  angle = ((msb & 0b00001111) << 8) + lsb;

  return angle;  
}

boolean AS5047D::update_current_position(int microsteps) 
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
