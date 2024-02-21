/*
    This file is part of the Arduino_Alvik library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#include "Arduino_Alvik.h"

Arduino_Alvik alvik;


float reference = 10.0;
float error = 0.0;
float distances[5];



void setup(){
  while((!Serial) && (millis()<3000));
  alvik.begin();
}

void loop(){
  alvik.get_distance(distances[0], distances[1], distances[2], distances[3], distances[4]);
  Serial.println(distances[2]);
  error = distances[2] - reference;
  alvik.set_wheels_speed(error * 10.0, error * 10.0);
  delay(100);
}

