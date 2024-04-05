/*
    This file is part of the Arduino_Alvik library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


/*
  You can use this sketch to make your Arduino Nano ESP32 an USB-to-serial adapter.
  This allows to flash firmware into the Arduino Alvik Carrier via Arduino Nano ESP32.

  Please refer to Arduino_AlvikCarrier library available at https://github.com/arduino-libraries/Arduino_AlvikCarrier

*/




#include "Arduino_Alvik.h"

unsigned long baud = 115200;

int rts = -1;
int dtr = -1;


static void onLineChange(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  if (event_base == ARDUINO_USB_CDC_EVENTS) {
    arduino_usb_cdc_event_data_t * data = (arduino_usb_cdc_event_data_t*)event_data;
    switch (event_id) {
      case ARDUINO_USB_CDC_LINE_CODING_EVENT:
        auto baud = data->line_coding.bit_rate;
        Serial0.updateBaudRate(baud);
        while (Serial0.available()) {
          Serial0.read();
        }
        break;
    }
  }
}

void setup() {
  Serial.onEvent(onLineChange);
  Serial.enableReboot(false);
  Serial.begin(baud);
  Serial.setRxBufferSize(0);
  Serial.setRxBufferSize(2048);
  Serial0.setRxBufferSize(8192);
  Serial0.setTxBufferSize(8192);
  Serial0.begin(baud, SERIAL_8E1);
  Serial0.flush();
  pinMode(BOOT_STM32, OUTPUT);
  pinMode(RESET_STM32, OUTPUT);
  digitalWrite(BOOT_STM32,HIGH);
  delay(1000);
  digitalWrite(RESET_STM32,LOW);
  delay(1000);
  digitalWrite(RESET_STM32,HIGH);
}

void loop() {
  int len = 0;
  uint8_t auc_buffer[488];
  while (Serial.available() && len < sizeof(auc_buffer)) {
    auc_buffer[len++] = Serial.read();
  }
  if (len) {
    Serial0.write(auc_buffer, len);
  }

  len = 0;
  while (Serial0.available() && len < sizeof(auc_buffer)) {
    auc_buffer[len++] = Serial0.read();
  }
  if (len) {
    Serial.write(auc_buffer, len);
  }
}