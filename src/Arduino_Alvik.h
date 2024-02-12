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




    void reset_hw(){
      digitalWrite(RESET_STM32, LOW);
      delay(100);
      digitalWrite(RESET_STM32, HIGH);
      delay(100);
    }

  public:
    HardwareSerial * uart;

    Arduino_Alvik(){
      update_semaphore = xSemaphoreCreateMutex();
      uart = new HardwareSerial(UART);
      packeter = new ucPack(200);
      version_semaphore = xSemaphoreCreateMutex();
      line_semaphore = xSemaphoreCreateMutex();
      color_semaphore = xSemaphoreCreateMutex();
      orientation_semaphore = xSemaphoreCreateMutex();
      imu_semaphore = xSemaphoreCreateMutex();
      distance_semaphore = xSemaphoreCreateMutex();
      touch_semaphore = xSemaphoreCreateMutex();
      joint_vel_semaphore = xSemaphoreCreateMutex();
      joint_pos_semaphore = xSemaphoreCreateMutex();
      robot_vel_semaphore = xSemaphoreCreateMutex();
      robot_pos_semaphore = xSemaphoreCreateMutex();
    }

    int begin(){
      last_ack = 0;

      version[0] = 0;
      version[1] = 0;
      version[2] = 0;

      led_state = 0;

      line_sensors[0] = 0;
      line_sensors[1] = 0;
      line_sensors[2] = 0;

      color_sensor[0] = 0;
      color_sensor[1] = 0;
      color_sensor[2] = 0;

      orientation[0] = 0.0;
      orientation[1] = 0.0;
      orientation[2] = 0.0;

      imu[0] = 0.0;
      imu[1] = 0.0;
      imu[2] = 0.0;
      imu[3] = 0.0;
      imu[4] = 0.0;
      imu[5] = 0.0;

      distances[0] = 0.0;
      distances[1] = 0.0;
      distances[2] = 0.0;
      distances[3] = 0.0;
      distances[4] = 0.0;
      distances[5] = 0.0;
      distances[6] = 0.0;

      touch = 0;
      touch_bits = 0;

      joints_velocity[0] = 0.0;
      joints_velocity[1] = 0.0;

      joints_position[0] = 0.0;
      joints_position[1] = 0.0;

      robot_velocity[0] = 0.0;
      robot_velocity[1] = 0.0;

      robot_pose[0] = 0.0;
      robot_pose[1] = 0.0;
      robot_pose[2] = 0.0;




      uart->begin(UART_BAUD_RATE);
      uart->flush();

      
      pinMode(CHECK_STM32, INPUT_PULLUP);
      pinMode(RESET_STM32, OUTPUT);

      if (digitalRead(CHECK_STM32)==0){
        return -1;
      }

      //begin_update_thread();
      xTaskCreatePinnedToCore(this->update_thread, "update", 10000, this, 1, &update_task, 0);

      delay(100);
      reset_hw();
      delay(1000);


      while (last_ack!=0x00){
        delay(20);
      }

      delay(2000);

      set_illuminator(true);


      return 0;
    }

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



    // return first available packet
    bool read_message(){  //must be private
      while (uart->available()){
        b = uart->read();
        packeter->buffer.push(b);
        if (packeter->checkPayload()){
          return true;
        }
      }
      return false;
    }

    // robot commands logic
    int parse_message(){   // must be private
      code = packeter->payloadTop();
      switch(code){
        // get joints velocity in RPM
        case 'j':
          while (!xSemaphoreTake(joint_vel_semaphore, 5)){}
          packeter->unpacketC2F(code, joints_velocity[0], joints_velocity[1]);
          xSemaphoreGive(joint_vel_semaphore);
          break;

        // get joints position in degrees
        case 'w':
          while (!xSemaphoreTake(joint_pos_semaphore, 5)){}
          packeter->unpacketC2F(code, joints_position[0], joints_position[1]);
          xSemaphoreGive(joint_pos_semaphore);
          break;

        // get robot linear and angular velocities in mm/s and degrees/s
        case 'v':
          while (!xSemaphoreTake(robot_vel_semaphore, 5)){}
          packeter->unpacketC2F(code, robot_velocity[0], robot_velocity[1]);
          xSemaphoreGive(robot_vel_semaphore);
          break;

        // get robot pose in mm and degrees, x, y, theta
        case 'z':
          while (!xSemaphoreTake(robot_pos_semaphore, 5)){}
          packeter->unpacketC3F(code, robot_pose[0], robot_pose[1], robot_pose[2]);
          xSemaphoreGive(robot_pos_semaphore);
          break;

        





        // get ack code
        case 'x':
            packeter->unpacketC1B(code, last_ack);
            break;

        // get line follower sensors, low is white - high is black: Left, Center, Right
        case 'l':
            while (!xSemaphoreTake(line_semaphore, 5)){}
            packeter->unpacketC3I(code, line_sensors[0], line_sensors[1], line_sensors[2]);
            xSemaphoreGive(line_semaphore);
            break;

        // get colors: red, green, blue
        case 'c':
            while (!xSemaphoreTake(color_semaphore, 5)){}
            packeter->unpacketC3I(code, color_sensor[0], color_sensor[1], color_sensor[2]);
            xSemaphoreGive(color_semaphore);
            break;
        
        // get orientation in deg: roll, pitch, yaw
        case 'q':
            while (!xSemaphoreTake(orientation_semaphore, 5)){}
            packeter->unpacketC3F(code, orientation[0], orientation[1], orientation[2]);
            xSemaphoreGive(orientation_semaphore);
            break;

        // get imu data in g and deg/s: aX, aY, aZ, gX, gY, gZ
        case 'i':
            while (!xSemaphoreTake(imu_semaphore, 5)){}
            packeter->unpacketC6F(code, imu[0], imu[1], imu[2], imu[3], imu[4], imu[5]);
            xSemaphoreGive(imu_semaphore);
            break;       
        
        // get data from ToF in mm: L, CL, C, CR, R, B, T
        case 'f':
            while (!xSemaphoreTake(distance_semaphore, 5)){}
            packeter->unpacketC7I(code, distances[0], distances[1], distances[2], distances[3], distances[4], distances[5], distances[6]);
            xSemaphoreGive(distance_semaphore);
            break;    

        case 't':
            while (!xSemaphoreTake(touch_semaphore, 5)){}
            packeter->unpacketC1B(code, touch);
            xSemaphoreGive(touch_semaphore);
            break;   
        
        
        // get version: Up, Mid, Low
        case 0x7E:
          while (!xSemaphoreTake(version_semaphore, 5)){}
          packeter->unpacketC3B(code, version[0], version[1], version[2]);
          xSemaphoreGive(version_semaphore);
          break;

        // nothing is parsed, the command is newer to this library
        default:
          return -1;
      }
      return 0;
    }

    void get_version(uint8_t & upper, uint8_t & middle, uint8_t & lower){
      while (!xSemaphoreTake(version_semaphore, 5)){}
      upper = version[0];
      middle = version[1];
      lower = version[2];
      xSemaphoreGive(version_semaphore);
    }

    void get_line_sensors(int16_t & left, int16_t & center, int16_t & right){
      while (!xSemaphoreTake(line_semaphore, 5)){}
      left = line_sensors[0];
      center = line_sensors[1];
      right = line_sensors[2];
      xSemaphoreGive(line_semaphore);
    }

    void get_color_raw(int16_t & red, int16_t & green, int16_t & blue){
      while (!xSemaphoreTake(color_semaphore, 5)){}
      red = color_sensor[0];
      green = color_sensor[1];
      blue = color_sensor[2];
      xSemaphoreGive(color_semaphore);
    }

    void get_orientation(float & roll, float & pitch, float & yaw){
      while (!xSemaphoreTake(orientation_semaphore, 5)){}
      roll = orientation[0];
      pitch = orientation[1];
      yaw = orientation[2];
      xSemaphoreGive(orientation_semaphore);
    }

    void get_accelerations(float & x, float & y, float & z){
      while (!xSemaphoreTake(imu_semaphore, 5)){}
      x = imu[0];
      y = imu[1];
      z = imu[2];
      xSemaphoreGive(imu_semaphore);
    }

    void get_gyros(float & x, float & y, float & z){
      while (!xSemaphoreTake(imu_semaphore, 5)){}
      x = imu[3];
      y = imu[4];
      z = imu[5];
      xSemaphoreGive(imu_semaphore);
    }

    void get_imu(float & ax, float & ay, float & az, float & gx, float & gy, float & gz){
      while (!xSemaphoreTake(imu_semaphore, 5)){}
      ax = imu[0];
      ay = imu[1];
      az = imu[2];
      gx = imu[3];
      gy = imu[4];
      gz = imu[5];
      xSemaphoreGive(imu_semaphore);
    }

    void get_distance(int16_t & left, int16_t & center_left, int16_t & center, int16_t & center_right, int16_t & right){
      while (!xSemaphoreTake(distance_semaphore, 5)){}
      left = distances[0];
      center_left = distances[1];
      center = distances[2];
      center_right = distances[3];
      right = distances[4];
      xSemaphoreGive(distance_semaphore);
    }

    void get_touch(){     //must be private
      while (!xSemaphoreTake(touch_semaphore, 5)){}
      touch_bits = touch;
      xSemaphoreGive(touch_semaphore);
    }

    bool get_touch_any(){
      get_touch();
      return touch_bits & 0b00000001;
    }

    bool get_touch_ok(){
      get_touch();
      return touch_bits & 0b00000010;
    }

    bool get_touch_cancel(){
      get_touch();
      return touch_bits & 0b00000100;
    }

    bool get_touch_center(){
      get_touch();
      return touch_bits & 0b00001000;
    }

    bool get_touch_up(){
      get_touch();
      return touch_bits & 0b00010000;
    }

    bool get_touch_left(){
      get_touch();
      return touch_bits & 0b00100000;
    }

    bool get_touch_down(){
      get_touch();
      return touch_bits & 0b01000000;
    }

    bool get_touch_right(){
      get_touch();
      return touch_bits & 0b10000000;
    }

    uint8_t get_ack(){
      return last_ack;
    }


    void get_wheels_speed(float & left, float & right){
      while (!xSemaphoreTake(joint_vel_semaphore, 5)){}
      left = joints_velocity[0];
      right = joints_velocity[1];
      xSemaphoreGive(joint_vel_semaphore);
    }
    
    void set_wheels_speed(const float left, const float right){
      msg_size = packeter->packetC2F('J', left, right);
      uart->write(packeter->msg, msg_size);
    }

    void get_wheels_position(float & left, float & right){
      while (!xSemaphoreTake(joint_pos_semaphore, 5)){}
      left = joints_position[0];
      right = joints_position[1];
      xSemaphoreGive(joint_pos_semaphore);
    }

    void set_wheels_position(const float left, const float right){
      msg_size = packeter->packetC2F('A', left, right);
      uart->write(packeter->msg, msg_size);
    }

    void get_drive_speed(float & linear, float & angular){
      while (!xSemaphoreTake(robot_vel_semaphore, 5)){}
      linear = robot_velocity[0];
      angular = robot_velocity[1];
      xSemaphoreGive(robot_vel_semaphore);
    }

    void drive(const float linear, const float angular){
      msg_size = packeter->packetC2F('V', linear, angular);
      uart->write(packeter->msg, msg_size);
    }

    void get_pose(float & x, float & y, float & theta){
      while (!xSemaphoreTake(robot_pos_semaphore, 5)){}
      x = robot_pose[0];
      y = robot_pose[1];
      theta = robot_pose[2];
      xSemaphoreGive(robot_pos_semaphore);
    }
    





















    void set_leds(){   // must be private
      msg_size = packeter->packetC1B('L', led_state);
      uart->write(packeter->msg, msg_size);
    }

    void set_builtin_led(bool value){
      if (value){
        led_state |= 1<<0;
      }
      else{
        led_state &= ~1<<0;
      }
      set_leds();
    }

    void set_illuminator(bool value){
      if (value){
        led_state |= 1<<1;
      }
      else{
        led_state &= ~1<<1;
      }
      set_leds();
    }

    void set_servo_positions(const uint8_t a_position, const uint8_t b_position){
      msg_size = packeter->packetC2B('S', a_position, b_position);
      uart->write(packeter->msg, msg_size);
    }

    void update(const int delay_value = 1){  //must be private
      while (update_task != NULL){
        if (read_message()){
          parse_message();
        }
        delay(delay_value);
      }
    }

    static void update_thread(void * pvParameters){  // must be private
      ((Arduino_Alvik*) pvParameters)->update();
    }

};



#endif