/*
    This file is part of the Arduino_Alvik library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

// This examples shows how to create a thread. When Alvik is lifted, it stops.

#include "Arduino_Alvik.h"

Arduino_Alvik alvik;

TaskHandle_t lift_task = NULL;


uint8_t color_val=0;

void setup() {
  alvik.begin();
  xTaskCreate(&check_lift, "lift_task", 10000, NULL, 0, &lift_task);
}

void loop() {
  blinking_leds(color_val);
  color_val = (color_val + 1) % 7;
  delay(500);
}

void blinking_leds(uint8_t val){
  alvik.left_led.set_color(val & 0x01, val & 0x02, val & 0x04);
  alvik.right_led.set_color(val & 0x02, val & 0x04, val & 0x01);
}

void check_lift(void * pvParameters){
  while(1){
    if (alvik.get_lifted()){
      alvik.brake();
    }
    else{
      alvik.drive(5,0);
    }
  }
}