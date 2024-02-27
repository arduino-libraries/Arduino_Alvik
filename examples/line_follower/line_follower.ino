/*
    This file is part of the Arduino_Alvik library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#include "Arduino_Alvik.h"

Arduino_Alvik alvik;

int line_sensors[3];
float error = 0;
float control = 0;
float kp = 50.0;



void setup() {
  Serial.begin(115200);
  while((!Serial)&&(millis()>3000));
  alvik.begin();
  alvik.left_led.set_color(0,0,1);
  alvik.right_led.set_color(0,0,1);

  while(!alvik.get_touch_ok()){
    delay(50);
  }

  alvik.left_led.set_color(0,1,0);
  alvik.right_led.set_color(0,1,0);

}

void loop() {
  while (!alvik.get_touch_cancel()){

    alvik.get_line_sensors(line_sensors[0], line_sensors[1], line_sensors[2]);
    Serial.print(line_sensors[0]);
    Serial.print("\t");
    Serial.print(line_sensors[1]);
    Serial.print("\t");
    Serial.print(line_sensors[2]);
    Serial.print("\n");
    error = calculate_center(line_sensors[0], line_sensors[1], line_sensors[2]);
    control = error * kp;
    if (control > 0.2){
      alvik.left_led.set_color(1,0,0);
      alvik.right_led.set_color(0,0,0);
    }
    else{
      if (control < -0.2){
        alvik.left_led.set_color(0,0,0);
        alvik.right_led.set_color(1,0,0);
      }
      else{
        alvik.left_led.set_color(0,1,0);
        alvik.right_led.set_color(0,1,0);
      }
    }

    alvik.set_wheels_speed(30-control, 30+control);
    delay(100);
  }
  while (!alvik.get_touch_ok()){
    alvik.left_led.set_color(0,0,1);
    alvik.right_led.set_color(0,0,1);
    alvik.brake();
    delay(100);
  }
}

float calculate_center(const int left, const int center, const int right){
  float centroid = 0.0; 
  float sum_weight = left + center + right;
  float sum_values = left + center * 2 + right * 3;
  if (sum_weight!=0.0){                                                         // divide by zero protection
    centroid=sum_values/sum_weight;
    centroid=-centroid+2.0;                                                     // so it is right on robot axis Y
  }
  return centroid;
}
