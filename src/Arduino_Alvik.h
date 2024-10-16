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
#include "definitions.h"
#include <EEPROM.h>
#include "default_colors.h"
#include "Wire.h"


class Arduino_Alvik{
  private:
    SemaphoreHandle_t update_semaphore;
    TaskHandle_t update_task;

    HardwareSerial * uart;

    bool verbose_output;

    uint8_t b;
    uint8_t code;
    ucPack * packeter;
    uint8_t msg_size;

    uint8_t last_ack;
    uint8_t waiting_ack;

    float converted_angular;


    SemaphoreHandle_t version_semaphore;
    uint8_t fw_version[3];
    uint8_t lib_version[3];

    uint8_t led_state;

    SemaphoreHandle_t line_semaphore;
    int16_t line_sensors[3];

    SemaphoreHandle_t color_semaphore;
    int16_t color_sensor[3];
    uint16_t white_cal[3];
    uint16_t black_cal[3];
    float rgb_normalized[3];
    float hsv[3];

    uint8_t servo_positions[2];

    SemaphoreHandle_t orientation_semaphore;
    float orientation[3];
    uint8_t move_bits;

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

    float battery;
    float battery_soc;
    uint16_t battery_val = 0;
    uint8_t battery_v[2];
    bool battery_is_charging;



    void reset_hw();                                                    // reset the robot
    void wait_for_ack();
    bool wait_for_fw_check();

    bool read_message();                                                // return first available packet
    int parse_message();                                                // robot commands logic
    void update(const int delay_value = 1);                             // thread to update data
    static void update_thread(void * pvParameters);                     // freertos thread

    void get_touch();                                                   // service function to parse touch
    void set_leds();                                                    // service function to set leds by a byte
    void wait_for_target(const int idle_time);                                             // service function that wait for ack

    float limit(float value, const float min, const float max);         // limit a value
    float normalize(float value, const float min, const float max);     // normalize a value
    void load_color_calibration();                                      // service function to get data from eeprom

    float battery_measure();                                            // service function to get data from bms



    class ArduinoAlvikRgbLed{                                           // service class for RGB led
      private:
        HardwareSerial * _serial;
        ucPack * _packeter;
        uint8_t * _led_state;
        uint8_t _offset;
        uint8_t _msg_size;
      public:
        String label;

        ArduinoAlvikRgbLed(){};
        ArduinoAlvikRgbLed(HardwareSerial * serial, ucPack * packeter, String label, uint8_t * led_state, uint8_t offset);
        void operator=(const ArduinoAlvikRgbLed& other);
        void set_color(const bool red, const bool green, const bool blue);
    };


    class ArduinoAlvikWheel{                                            // service class for wheel
      private:
        HardwareSerial * _serial;
        ucPack * _packeter;
        uint8_t _msg_size;
        float _wheel_diameter;
        uint8_t _label;
        float * _joint_velocity;
        float * _joint_position;
        float converted_vel;
        Arduino_Alvik * _alvik;
      public:
        ArduinoAlvikWheel():_alvik(nullptr){};
        ArduinoAlvikWheel(HardwareSerial * serial, ucPack * packeter, uint8_t label, 
                          float * joint_velocity, float * joint_position, float wheel_diameter,
                          Arduino_Alvik & alvik);

        void reset(const float initial_position = 0.0, const uint8_t unit = DEG);

        void set_pid_gains(const float kp, const float ki, const float kd);

        void stop();
        void set_speed(const float velocity, const uint8_t unit = RPM);
        float get_speed(const uint8_t unit = RPM);

        void set_position(const float position, const uint8_t unit = DEG, const bool blocking = true);
        float get_position(const uint8_t unit = DEG);
    };

    class ArduinoAlvikServo{
      private:
        HardwareSerial * _serial;
        ucPack * _packeter;
        uint8_t _msg_size;
        uint8_t _label;
        uint8_t _servo_id;
        uint8_t * _positions;
      public:
        ArduinoAlvikServo(){};
        ArduinoAlvikServo(HardwareSerial * serial, ucPack * packeter, char label, uint8_t servo_id, uint8_t * positions);
        void set_position(const uint8_t position);
        int get_position();
    };


  public:
    TwoWire & i2c;
    Arduino_Alvik::ArduinoAlvikRgbLed left_led;
    Arduino_Alvik::ArduinoAlvikRgbLed right_led;
    Arduino_Alvik::ArduinoAlvikWheel left_wheel;
    Arduino_Alvik::ArduinoAlvikWheel right_wheel;
    Arduino_Alvik::ArduinoAlvikServo servo_A;
    Arduino_Alvik::ArduinoAlvikServo servo_B;
    String COLOR_LABELS[13] = {"black", "grey", "light grey", "white",
                        "yellow", "light_green", "green",
                        "light_blue", "blue", "violet",
                        "brown", "orange", "red"};

    Arduino_Alvik();

    int begin(const bool verbose = true, const uint8_t core = RUN_ON_CORE_0);
    void stop();
    bool is_on();
    void idle();


    uint8_t get_ack();


    void get_wheels_speed(float & left, float & righ, const uint8_t unit = RPM);
    void set_wheels_speed(const float left, const float right, const uint8_t unit = RPM);

    void get_wheels_position(float & left, float & right, const uint8_t unit = DEG);
    void set_wheels_position(const float left, const float right, const uint8_t unit = DEG, const bool blocking = true);

    void get_drive_speed(float & linear, float & angular, const uint8_t linear_unit = CM_S, const uint8_t angular_unit = DEG_S);
    void drive(const float linear, const float angular, const uint8_t linear_unit = CM_S, const uint8_t angular_unit = DEG_S);

    void get_pose(float & x, float & y, float & theta, const uint8_t distance_unit = CM, const uint8_t angle_unit = DEG);
    void reset_pose(const float x = 0.0, const float y = 0.0, const float theta = 0.0, const uint8_t distance_unit = CM, const uint8_t angle_unit = DEG);

    bool is_target_reached();
    void rotate(const float angle, const uint8_t unit = DEG, const bool blocking = true);
    void move(const float distance, const uint8_t unit = CM, const bool blocking = true);

    void brake();
    

    void get_line_sensors(int & left, int & center, int & right);
    
    void get_color_raw(int & red, int & green, int & blue);
    void rgb2norm(const int16_t r, const int16_t g, const int16_t b, float & r_norm, float & g_norm, float & b_norm);
    void norm2hsv(const float r, const float g, const float b, float & h, float & s, float & v);
    void get_color(float & value0, float & value1, float & value2, const uint8_t format = RGB);
    uint8_t get_color_id(const float h, const float s, const float v);
    uint8_t get_color_id();
    String get_color_label(const float h, const float s, const float v);
    String get_color_label();
    void color_calibration(const uint8_t background = WHITE_ID);

    void get_orientation(float & roll, float & pitch, float & yaw);
    void get_accelerations(float & x, float & y, float & z);
    void get_gyros(float & x, float & y, float & z);
    void get_imu(float & ax, float & ay, float & az, float & gx, float & gy, float & gz);
    bool get_shake();
    String get_tilt();

    void get_distance(float & left, float & center_left, float & center, float & center_right, float & right, const uint8_t unit = CM);
    float get_distance_top(const uint8_t unit = CM);
    float get_distance_bottom(const uint8_t unit = CM);


    bool get_touch_any();
    bool get_touch_ok();
    bool get_touch_cancel();
    bool get_touch_center();
    bool get_touch_up();
    bool get_touch_left();
    bool get_touch_down();
    bool get_touch_right();


    void set_builtin_led(const bool value);
    void set_illuminator(const bool value);

    void set_servo_positions(const uint8_t a_position, const uint8_t b_position);
    void get_servo_positions(int & a_position, int & b_position);

    void set_behaviour(const uint8_t behaviour);
    
    void get_version(uint8_t & upper, uint8_t & middle, uint8_t & lower, const String version="fw");
    void get_fw_version(uint8_t & upper, uint8_t & middle, uint8_t & lower);
    void get_lib_version(uint8_t & upper, uint8_t & middle, uint8_t & lower);
    void get_required_fw_version(uint8_t & upper, uint8_t & middle, uint8_t & lower);
    bool check_firmware_compatibility();

    int get_battery_charge();
    bool is_battery_charging();
};



#endif


/*
     _            _       _             
    / \   _ __ __| |_   _(_)_ __   ___  
   / _ \ | '__/ _` | | | | | '_ \ / _ \ 
  / ___ \| | | (_| | |_| | | | | | (_) |
 /_/   \_\_|  \__,_|\__,_|_|_| |_|\___/ 
     _    _       _ _                   
    / \  | |_   _(_) | __               
   / _ \ | \ \ / / | |/ /               
  / ___ \| |\ V /| |   <                
 /_/   \_\_| \_/ |_|_|\_\   

+---+----------------------------+--+----------+
|   |      ()             ()     |  |          |
|   |            \__/            |  |   /---\  |
|    \__________________________/   |   |   |  |
|                                   |   |   |  |
+-----------------------------------+---|   |--+
   \\\___/                            \\\___/   
*/
