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

#define AS5047D_CMD_NOP   (0x0000)
#define AS5047D_CMD_ERRFL (0x0001)
#define AS5047D_CMD_PROG  (0x0003)
#define AS5047D_CMD_DIAAGC (0x3FFC)
#define AS5047D_CMD_MAG    (0x3FFD)
#define AS5047D_CMD_ANGLEUNC (0x3FFE)
#define AS5047D_CMD_ANGLECOM (0x3FFF)


#define AS5048A_CMD_NOP   (0x0000)
#define AS5048A_CMD_ERRFL (0x0001)
#define AS5048A_CMD_PROG  (0x0003)
#define AS5048A_CMD_DIAAGC (0x3FFD)
#define AS5048A_CMD_MAG    (0x3FFE)
#define AS5048A_CMD_ANGLE  (0x3FFF)

#define RD  0x40    // bit 14 "1" is Read + parity even

#pragma GCC push_options
#pragma GCC optimize ("-Ofast")

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

void init_position_sensor()
{
	digitalWrite(PIN_AS5047D_CS,LOW); //pull CS LOW by default (chip powered off)
	digitalWrite(PIN_MOSI,LOW);
	digitalWrite(PIN_SCK,LOW);
	digitalWrite(PIN_MISO,LOW);
	pinMode(PIN_MISO,OUTPUT);
	delay(1000);

	digitalWrite(PIN_AS5047D_CS,HIGH); //pull CS hig
  
	pinMode(PIN_MISO,INPUT);

	error=false;
	SPISettings settingsA(5000000, MSBFIRST, SPI_MODE1);   //400000, MSBFIRST, SPI_MODE1;

	pinMode(PIN_AS5047D_CS,OUTPUT);
	digitalWrite(PIN_AS5047D_CS,HIGH);                     //pull CS high by default
	delay(1);
	SPI.begin();                                          //AS5047D SPI uses mode=1 (CPOL=0, CPHA=1)

	SPI.beginTransaction(settingsA);
	SPI.transfer16(AS5047D_CMD_NOP);
	delay(10);

	//wait for the LF bit to be set
	uint16_t data=0,t0=100;
	while (getBit(data,8)==0 && t0>0)
	{
		delay(1);
		t0--;
		if (t0==0)
		{
			ERROR("LF bit not set");
			error=true;
			break;
			//return false;
		}
		data=readAddress(AS5047D_CMD_DIAAGC);
	}

	if (error)
	{
		error=false;
		uint16_t data=0,t0=100;
		while (getBit(data,8)==0 && t0>0)
		{
			delay(1);
			t0--;
			if (t0==0)
			{
				ERROR("AS5048A OCF bit not set");
				error=true;
				return false;
			}
			data=readAddress(AS5048A_CMD_DIAAGC);
			LOG("AS5048A diag data is 0x%04X",data);
		}
		as5047d=false;
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
		ERROR("read command 0x%04X failed",addr);
		error=true;
		return -1;
	}

	if (data>>15 != getParity(data))
	{
		//parity did not match
		ERROR("read command parity error 0x%04X ",addr);
		error=true;
		return -2;
	}

	data=data & 0x3FFF; //mask off the error and parity bits

	return data;
}

//read the encoders 
int16_t AS5047D::readEncoderAngle(void)
{
	if (as5047d)
	{
		return readAddress(AS5047D_CMD_ANGLECOM);
	}
	return readAddress(AS5048A_CMD_ANGLE);
}

//pipelined read of the encoder angle used for high speed reads, but value is always one read behind
int16_t AS5047D::readEncoderAnglePipeLineRead(void)
{

	int16_t data;
	int error, t0=10;
	GPIO_LOW(chipSelectPin);//(chipSelectPin, LOW);
	//delayMicroseconds(1);
	do {

		// doing two 8 bit transfers is faster than one 16 bit
		data =(uint16_t)SPI.transfer(0xFF)<<8 | ((uint16_t)SPI.transfer(0xFF) & 0x0FF);
		t0--;
		if (t0<=0)
		{
			ERROR("AS5047D problem");
			break;
		}
		//data=SPI.transfer16(0xFFFF); //to speed things up we know the parity and address for the read
	}while(data & (1<<14)); //while error bit is set

	data=data & 0x3FFF; //mask off the error and parity bits
	GPIO_HIGH(chipSelectPin);
	//digitalWrite(chipSelectPin, HIGH);
	//TODO we really should check for errors and return a negative result or something
	return data;
}


void AS5047D::diagnostics(char *ptrStr)
{
	int16_t data;
	int m,d;

	if (as5047d)
	{

	data=readAddress(AS5047D_CMD_DIAAGC);

	if (NULL == ptrStr)
	{
		LOG("DIAAGC: 0x%04X", data);
		LOG("MAGL: %d", getBit(data,11));
		LOG("MAGH: %d", getBit(data,10));
		LOG("COF: %d", getBit(data,9));
		LOG("LFGL: %d", getBit(data,8));
		LOG("AGC: %d", data & 0x0FF);

		data=readAddress(AS5047D_CMD_MAG);
		LOG("CMAG: 0x%04X(%d)",data,data);

		data=readAddress(AS5047D_CMD_ANGLEUNC);
		m=(int)((float)data*AS5047D_DEGREES_PER_BIT);
		d=(int)((float)data*AS5047D_DEGREES_PER_BIT*100 -m*100);
		LOG("CORDICANG: 0x%04X(%d) %d.%02d deg(est)",data,data,m,d);

		data=readAddress(AS5047D_CMD_ANGLECOM);
		m=(int)((float)data*AS5047D_DEGREES_PER_BIT);
		d=(int)((float)data*AS5047D_DEGREES_PER_BIT*100 -m*100);
		LOG("DAECANG: 0x%04X(%d) %d.%02d deg(est)",data,data,m,d);
	}else
	{
		sprintf(ptrStr,"DIAAGC: 0x%04X\n\r", data);
		sprintf(ptrStr,"%sMAGL: %d\n\r", ptrStr,getBit(data,11));
		sprintf(ptrStr,"%sMAGH: %d\n\r", ptrStr,getBit(data,10));
		sprintf(ptrStr,"%sCOF: %d\n\r", ptrStr, getBit(data,9));
		sprintf(ptrStr,"%sLFGL: %d\n\r", ptrStr, getBit(data,8));
		sprintf(ptrStr,"%sAGC: %d\n\r", ptrStr,data & 0x0FF);

		data=readAddress(AS5047D_CMD_MAG);
		sprintf(ptrStr,"%sCMAG: 0x%04X(%d)\n\r", ptrStr,data,data);

		data=readAddress(AS5047D_CMD_ANGLEUNC);
		m=(int)((float)data*AS5047D_DEGREES_PER_BIT);
		d=(int)((float)data*AS5047D_DEGREES_PER_BIT*100 -m*100);
		sprintf(ptrStr,"%sCORDICANG: 0x%04X(%d) %d.%02d deg(est)\n\r", ptrStr,data,data,m,d);

		data=readAddress(AS5047D_CMD_ANGLECOM);
		m=(int)((float)data*AS5047D_DEGREES_PER_BIT);
		d=(int)((float)data*AS5047D_DEGREES_PER_BIT*100 -m*100);
		sprintf(ptrStr,"%sDAECANG: 0x%04X(%d) %d.%02d deg(est)\n\r", ptrStr,data,data,m,d);

	}
	} else
	{
		data=readAddress(AS5048A_CMD_DIAAGC);
		sprintf(ptrStr,"AS5048A DIAAGC: 0x%04X\n\r", data);
		data=readAddress(AS5048A_CMD_MAG);
		sprintf(ptrStr,"%sMagnitude: %d\n\r", ptrStr,data);
		data=readAddress(AS5048A_CMD_ANGLE);
		sprintf(ptrStr,"%sAngle: %d\n\r", ptrStr,data);
	}

}

// ************************Write to AS5047D **************************
void AS5047D_Write( int SSPin, int address, int value)
{
  // take the SS pin low to select the chip:
  digitalWrite(SSPin, LOW);
  
  Serial.println(value, HEX);
  
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
