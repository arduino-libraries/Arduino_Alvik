/*
    This file is part of the Arduino_Alvik library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/

#ifndef __ARDUINO_ALVIK_H__
#define __ARDUINO_ALVIK_H__

#include "Arduino.h"
#include "ucPack.h"

#define CHECK_STM32 A6
#define BOOT_STM32 D2
#define RESET_STM32 D3
#define UART 0
#define UART_BAUD_RATE 115200

class Arduino_Alvik{
  private:
    SemaphoreHandle_t update_semaphore;
    TaskHandle_t update_task;




    uint8_t b;
    uint8_t code;
    ucPack * packeter;
    uint8_t msg_size;

    uint8_t last_ack;



    SemaphoreHandle_t version_semaphore;
    uint8_t version[3];

    uint8_t led_state;

    SemaphoreHandle_t line_semaphore;
    int16_t line_sensors[3];

    SemaphoreHandle_t color_semaphore;
    int16_t color_sensor[3];

    SemaphoreHandle_t orientation_semaphore;
    float orientation[3];

    SemaphoreHandle_t imu_semaphore;
    float imu[6];

    SemaphoreHandle_t distance_semaphore;
    int16_t distances[7];

    SemaphoreHandle_t touch_semaphore;
    uint8_t touch, touch_bits;

    SemaphoreHandle_t joint_vel_semaphore;
    float joints_velocity[2];
    
    SemaphoreHandle_t joint_pos_semaphore;
    float joints_position[2];

    SemaphoreHandle_t robot_vel_semaphore;
    float robot_velocity[2];

    SemaphoreHandle_t robot_pos_semaphore;
    float robot_pose[3];



    void reset_hw();     // reset the robot
       
    bool read_message(); // return first available packet
    int parse_message(); // robot commands logic

    void get_touch();    // parse touch
    void set_leds();     // set leds by a byte


    void update(const int delay_value = 1);

    static void update_thread(void * pvParameters);


    class ArduinoAlvikRgbLed{
      private:
        HardwareSerial * _serial;
        ucPack * _packeter;
        uint8_t * _led_state;
        uint8_t _offset;
        uint8_t _msg_size;

      public:
        ArduinoAlvikRgbLed(){};
        ArduinoAlvikRgbLed(HardwareSerial * serial, ucPack * packeter, uint8_t * led_state, uint8_t offset){
          _serial = serial;
          _packeter = packeter;
          _led_state = led_state;
          _offset = offset;
        }

        void operator=(const ArduinoAlvikRgbLed& other){ 
          _serial = other._serial;
          _packeter = other._packeter;
          _led_state = other._led_state;
          _offset = other._offset;
          _msg_size = other._msg_size;

        }

        void set_color(const bool red, const bool green, const bool blue){
          if (red){
            (*_led_state) = (*_led_state) | (1<<_offset);
          }
          else{
            (*_led_state) = (*_led_state) & (~(1<<_offset));
          }

          if (green){
            (*_led_state) = (*_led_state) | (1<<(_offset+1));
          }
          else{
            (*_led_state) =  (*_led_state) & ~(1<<(_offset+1));
          }
          
          if (blue){
            (*_led_state) = (*_led_state) | (1<<(_offset+2));
          }
          else{
            (*_led_state) = (*_led_state) & ~(1<<(_offset+2));
          }

          _msg_size = _packeter->packetC1B('L', *_led_state);
          _serial->write(_packeter->msg, _msg_size);
        }
    };




  public:
    HardwareSerial * uart;
    Arduino_Alvik::ArduinoAlvikRgbLed left_led;
    Arduino_Alvik::ArduinoAlvikRgbLed right_led;

    Arduino_Alvik();

    int begin();

    /*
    void begin_update_thread(){
      if (xSemaphoreTake(update_semaphore, 5)){
        xTaskCreatePinnedToCore(this->update_thread, "update", 10000, this, 1, &update_task, 0);
      }
    }

    void stop_update_thread(){
      //update_task = NULL;
      xSemaphoreGive(update_semaphore);
    }

    void stop(){
      // stop wheels;
      stop_update_thread();
    }
    */


    void get_version(uint8_t & upper, uint8_t & middle, uint8_t & lower);

    void get_line_sensors(int16_t & left, int16_t & center, int16_t & right);

    void get_color_raw(int16_t & red, int16_t & green, int16_t & blue);

    void get_orientation(float & roll, float & pitch, float & yaw);
    void get_accelerations(float & x, float & y, float & z);
    void get_gyros(float & x, float & y, float & z);
    void get_imu(float & ax, float & ay, float & az, float & gx, float & gy, float & gz);


    void get_distance(int16_t & left, int16_t & center_left, int16_t & center, int16_t & center_right, int16_t & right);


    bool get_touch_any();
    bool get_touch_ok();
    bool get_touch_cancel();
    bool get_touch_center();
    bool get_touch_up();
    bool get_touch_left();
    bool get_touch_down();
    bool get_touch_right();


    uint8_t get_ack();


    void get_wheels_speed(float & left, float & right);
    void set_wheels_speed(const float left, const float right);

    void get_wheels_position(float & left, float & right);
    void set_wheels_position(const float left, const float right);

    void get_drive_speed(float & linear, float & angular);
    void drive(const float linear, const float angular);

    void get_pose(float & x, float & y, float & theta);
    



    void set_builtin_led(bool value);
    void set_illuminator(bool value);

    void set_servo_positions(const uint8_t a_position, const uint8_t b_position);
};



#endif