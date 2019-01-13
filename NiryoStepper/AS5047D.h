/*
    AS5047D.h
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

#include "config.h"
#include <Arduino.h>
#include <Wire.h>

extern volatile long sensor_position;
extern volatile long last_sensor_position;
extern volatile long sensor_position_with_rotations;
extern volatile long motor_rotation_count;

extern volatile long motor_position_without_offset;
extern volatile long motor_position_steps;

extern volatile long offset;

void AS5047D_setup();
int read_encoder();

void update_current_position(int microsteps);

void AS5047D_Write( int SSPin, int address, int value);
unsigned int AS5047D_Read( int SSPin, unsigned int address);

int parity(unsigned int x);
