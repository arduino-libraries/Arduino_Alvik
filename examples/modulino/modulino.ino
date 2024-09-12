/*
    This file is part of the Arduino_Alvik library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#include "Arduino_Alvik.h"
#include <Modulino.h>

Arduino_Alvik alvik;
ModulinoPixels leds;


void setup() {
  alvik.begin();
  Modulino.begin(alvik.i2c);        // pass here alvik.i2c
  leds.begin();
}

void loop() {
  for (int i = 0; i < 8; i++) {
    leds.clear();
    leds.set(i, 255, 0, 0, 100);
    leds.show();
    delay(100);
  }
  for (int i = 7; i >= 0; i--) {
    leds.clear();
    leds.set(i, 255, 0, 0, 100);
    leds.show();
    delay(100);
  }
}
