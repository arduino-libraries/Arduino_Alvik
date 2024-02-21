/*
    This file is part of the Arduino_Alvik library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#include "Arduino_Alvik.h"

Arduino_Alvik alvik;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  alvik.begin();

  Serial.println("Alvik initializated");
  Serial.println("Place your robot on white and send a char");
  while(Serial.available()==0);
  while(Serial.available()>0){
    Serial.read();
  }
  alvik.color_calibration(WHITE);
  Serial.println("Place your robot on black and send a char");
  while(Serial.available()==0);
    while(Serial.available()>0){
    Serial.read();
  }
  alvik.color_calibration(BLACK);
  
}

void loop() {
  int16_t color[3];
  float rgb[3];
  float hsv[3];
  alvik.get_color_raw(color[0], color[1], color[2]);
  alvik.get_color(rgb[0], rgb[1], rgb[2]);
  alvik.get_color(hsv[0], hsv[1], hsv[2], HSV);
  Serial.print(color[0]);
  Serial.print("\t");
  Serial.print(color[1]);
  Serial.print("\t");
  Serial.print(color[2]);
  Serial.print("\t");
  Serial.print(rgb[0]);
  Serial.print("\t");
  Serial.print(rgb[1]);
  Serial.print("\t");
  Serial.print(rgb[2]);
  Serial.print("\t");
  Serial.print(hsv[0]);
  Serial.print("\t");
  Serial.print(hsv[1]);
  Serial.print("\t");
  Serial.print(hsv[2]);
  Serial.print("\t");
  Serial.print(alvik.get_color_label(hsv[0], hsv[1], hsv[2]));
  Serial.print("\n");
}
