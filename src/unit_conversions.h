#ifndef __UNIT_CONVERSIONS_H__
#define __UNIT_CONVERSIONS_H__

#include "Arduino.h"
#include "definitions.h"


float convert_distance(const float value, const uint8_t from_unit, const uint8_t to_unit){           
  if ((to_unit<4)&&(from_unit<4)){
    return value*DISTANCE_UNITS[from_unit]/DISTANCE_UNITS[to_unit];
  }
  else{
    return value;
  }
}

float convert_speed(const float value, const uint8_t from_unit, const uint8_t to_unit){ 
  if ((to_unit<4)&&(from_unit<4)){
    return value*SPEED_UNITS[from_unit]/SPEED_UNITS[to_unit];
  }
  else{
    return value;
  }
}

float convert_angle(const float value, const uint8_t from_unit, const uint8_t to_unit){ 
  if ((to_unit<4)&&(from_unit<4)){
    return value*ANGLE_UNITS[from_unit]/ANGLE_UNITS[to_unit];
  }
  else{
    return value;
  }
}

float convert_rotational_speed(const float value, const uint8_t from_unit, const uint8_t to_unit){ 
  if ((to_unit<4)&&(from_unit<4)){
    return value*ROTATIONAL_SPEED_UNITS[from_unit]/ROTATIONAL_SPEED_UNITS[to_unit];
  }
  else{
    return value;
  }
}

#endif