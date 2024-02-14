/*
    This file is part of the Arduino_Alvik library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#include "Arduino_Alvik.h"
#include "unit_conversions.h"

Arduino_Alvik::Arduino_Alvik(){
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

  left_led = ArduinoAlvikRgbLed(uart, packeter, "left_led", &led_state, 2);
  right_led = ArduinoAlvikRgbLed(uart, packeter,"right_led", &led_state, 5);

  left_wheel = ArduinoAlvikWheel(uart, packeter, 'L', &joints_velocity[0], &joints_position[0]);
  right_wheel = ArduinoAlvikWheel(uart, packeter, 'R', &joints_velocity[1], &joints_position[1]);
}

void Arduino_Alvik::reset_hw(){                                                   //it is private
  digitalWrite(RESET_STM32, LOW);
  delay(100);
  digitalWrite(RESET_STM32, HIGH);
  delay(100);
}

int Arduino_Alvik::begin(){  
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
  set_behaviour(BEHAVIOUR_ILLUMINATOR_RISE);

  return 0;
}

void Arduino_Alvik::stop(){
  set_wheels_speed(0,0);
}


//-----------------------------------------------------------------------------------------------//
//                                          update                                               //
//-----------------------------------------------------------------------------------------------//

void Arduino_Alvik::update(const int delay_value){                                //it is private
  while (update_task != NULL){
    if (read_message()){
      parse_message();
    }
    delay(delay_value);
  }
}

void Arduino_Alvik::update_thread(void * pvParameters){                           //it is private
  ((Arduino_Alvik*) pvParameters)->update();
}

bool Arduino_Alvik::read_message(){                                               //it is private
  while (uart->available()){
    b = uart->read();
    packeter->buffer.push(b);
    if (packeter->checkPayload()){
      return true;
    }
  }
  return false;
}

int Arduino_Alvik::parse_message(){                                               //it is private
  code = packeter->payloadTop();
  switch(code){
    // get ack code
    case 'x':
      packeter->unpacketC1B(code, last_ack);
      break;


    // motion

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


    // sensors

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

    // get data from touch pads: any, ok, delete, center, left, down, right, up
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

uint8_t Arduino_Alvik::get_ack(){
  return last_ack;
}


//-----------------------------------------------------------------------------------------------//
//                                          motion                                               //
//-----------------------------------------------------------------------------------------------//

void Arduino_Alvik::get_wheels_speed(float & left, float & right, const uint8_t unit){
  while (!xSemaphoreTake(joint_vel_semaphore, 5)){}
  left = left_wheel.get_speed(unit);
  right = right_wheel.get_speed(unit);
  xSemaphoreGive(joint_vel_semaphore);
}

void Arduino_Alvik::set_wheels_speed(const float left, const float right, const uint8_t unit){
  msg_size = packeter->packetC2F('J', convert_rotational_speed(left, unit, RPM), convert_rotational_speed(right, unit, RPM));
  uart->write(packeter->msg, msg_size);
}

void Arduino_Alvik::get_wheels_position(float & left, float & right, const uint8_t unit){
  while (!xSemaphoreTake(joint_pos_semaphore, 5)){}
  left = convert_angle(joints_position[0], DEG, unit);
  right = convert_angle(joints_position[1], DEG, unit);
  xSemaphoreGive(joint_pos_semaphore);
}

void Arduino_Alvik::set_wheels_position(const float left, const float right, const uint8_t unit){
  msg_size = packeter->packetC2F('A', convert_angle(left, unit, DEG), convert_angle(right, unit, DEG));
  uart->write(packeter->msg, msg_size);
}

void Arduino_Alvik::get_drive_speed(float & linear, float & angular, const uint8_t linear_unit, const uint8_t angular_unit){
  while (!xSemaphoreTake(robot_vel_semaphore, 5)){}
  linear = convert_speed(robot_velocity[0], 'MM_S', linear_unit);
  if (angular_unit == PERCENTAGE){
    angular = (robot_velocity[1]/ROBOT_MAX_DEG_S)*100.0;
  }
  else{
    angular = convert_rotational_speed(robot_velocity[1], DEG_S, angular_unit);
  }  
  xSemaphoreGive(robot_vel_semaphore);
}

void Arduino_Alvik::drive(const float linear, const float angular, const uint8_t linear_unit, const uint8_t angular_unit){
  if (angular_unit == PERCENTAGE){
    converted_angular = (robot_velocity[1]/ROBOT_MAX_DEG_S)*100.0;
  }
  else{
    converted_angular = convert_rotational_speed(robot_velocity[1], DEG_S, angular_unit);
  } 
  msg_size = packeter->packetC2F('V', convert_speed(linear, linear_unit, MM_S), angular);
  uart->write(packeter->msg, msg_size);
}

void Arduino_Alvik::get_pose(float & x, float & y, float & theta, const uint8_t distance_unit, const uint8_t angle_unit){
  while (!xSemaphoreTake(robot_pos_semaphore, 5)){}
  x = convert_distance(robot_pose[0], MM, distance_unit);
  y = convert_distance(robot_pose[1], MM, distance_unit);
  theta = convert_angle(robot_pose[2], DEG, angle_unit);
  xSemaphoreGive(robot_pos_semaphore);
}

void Arduino_Alvik::reset_pose(const float x, const float y, const float theta, const uint8_t distance_unit, const uint8_t angle_unit){
  msg_size = packeter->packetC3F('Z', convert_distance(x, distance_unit, MM), convert_distance(y, distance_unit, MM), convert_distance(theta, angle_unit, DEG));
  uart->write(packeter->msg, msg_size); 
}

bool Arduino_Alvik::is_target_reached(){
  if ((last_ack != 'M') && (last_ack != 'R')){
    delay(50);
    return false;
  }
  msg_size = packeter->packetC1B('X', 'K');
  uart->write(packeter->msg, msg_size);
  delay(200);
  return true;
}

void Arduino_Alvik::wait_for_target(){                                             //it is private
  while (!is_target_reached()){}
}

void Arduino_Alvik::rotate(const float angle, const uint8_t unit, const bool blocking){
  delay(200);
  msg_size = packeter->packetC1F('R', convert_angle(angle, unit, DEG));
  uart->write(packeter->msg, msg_size);
  if (blocking){
    wait_for_target();
  }
}

void Arduino_Alvik::move(const float distance, const uint8_t unit, const bool blocking){
  delay(200);
  msg_size = packeter->packetC1F('G', convert_distance(distance, unit, MM));
  uart->write(packeter->msg, msg_size);
  if (blocking){
    wait_for_target();
  }
}


//-----------------------------------------------------------------------------------------------//
//                                              sensors                                          //
//-----------------------------------------------------------------------------------------------//

void Arduino_Alvik::get_line_sensors(int16_t & left, int16_t & center, int16_t & right){
  while (!xSemaphoreTake(line_semaphore, 5)){}
  left = line_sensors[0];
  center = line_sensors[1];
  right = line_sensors[2];
  xSemaphoreGive(line_semaphore);
}


void Arduino_Alvik::get_color_raw(int16_t & red, int16_t & green, int16_t & blue){
  while (!xSemaphoreTake(color_semaphore, 5)){}
  red = color_sensor[0];
  green = color_sensor[1];
  blue = color_sensor[2];
  xSemaphoreGive(color_semaphore);
}


void Arduino_Alvik::get_orientation(float & roll, float & pitch, float & yaw){
  while (!xSemaphoreTake(orientation_semaphore, 5)){}
  roll = orientation[0];
  pitch = orientation[1];
  yaw = orientation[2];
  xSemaphoreGive(orientation_semaphore);
}

void Arduino_Alvik::get_accelerations(float & x, float & y, float & z){
  while (!xSemaphoreTake(imu_semaphore, 5)){}
  x = imu[0];
  y = imu[1];
  z = imu[2];
  xSemaphoreGive(imu_semaphore);
}

void Arduino_Alvik::get_gyros(float & x, float & y, float & z){
  while (!xSemaphoreTake(imu_semaphore, 5)){}
  x = imu[3];
  y = imu[4];
  z = imu[5];
  xSemaphoreGive(imu_semaphore);
}

void Arduino_Alvik::get_imu(float & ax, float & ay, float & az, float & gx, float & gy, float & gz){
  while (!xSemaphoreTake(imu_semaphore, 5)){}
  ax = imu[0];
  ay = imu[1];
  az = imu[2];
  gx = imu[3];
  gy = imu[4];
  gz = imu[5];
  xSemaphoreGive(imu_semaphore);
}


void Arduino_Alvik::get_distance(float & left, float & center_left, float & center, float & center_right, float & right, const uint8_t unit){
  while (!xSemaphoreTake(distance_semaphore, 5)){}
  left = convert_distance(distances[0], MM, unit);
  center_left = convert_distance(distances[1], MM, unit);
  center = convert_distance(distances[2], MM, unit);
  center_right = convert_distance(distances[3], MM, unit);
  right = convert_distance(distances[4], MM, unit);
  xSemaphoreGive(distance_semaphore);
}

void Arduino_Alvik::get_distance_top(float & top, const uint8_t unit){
  top = convert_distance(distances[5], MM, unit);
}

void Arduino_Alvik::get_distance_bottom(float & bottom, const uint8_t unit){
  bottom = convert_distance(distances[6], MM, unit);
}


void Arduino_Alvik::get_touch(){                                                  //it is private
  while (!xSemaphoreTake(touch_semaphore, 5)){}
  touch_bits = touch;
  xSemaphoreGive(touch_semaphore);
}

bool Arduino_Alvik::get_touch_any(){
  get_touch();
  return touch_bits & 0b00000001;
}

bool Arduino_Alvik::get_touch_ok(){
  get_touch();
  return touch_bits & 0b00000010;
}

bool Arduino_Alvik::get_touch_cancel(){
  get_touch();
  return touch_bits & 0b00000100;
}

bool Arduino_Alvik::get_touch_center(){
  get_touch();
  return touch_bits & 0b00001000;
}

bool Arduino_Alvik::get_touch_up(){
  get_touch();
  return touch_bits & 0b00010000;
}

bool Arduino_Alvik::get_touch_left(){
  get_touch();
  return touch_bits & 0b00100000;
}

bool Arduino_Alvik::get_touch_down(){
  get_touch();
  return touch_bits & 0b01000000;
}

bool Arduino_Alvik::get_touch_right(){
  get_touch();
  return touch_bits & 0b10000000;
}


//-----------------------------------------------------------------------------------------------//
//                                   leds and peripherials                                       //
//-----------------------------------------------------------------------------------------------//

void Arduino_Alvik::set_leds(){                                                   //it is private
  msg_size = packeter->packetC1B('L', led_state);
  uart->write(packeter->msg, msg_size);
}

void Arduino_Alvik::set_builtin_led(const bool value){
  if (value){
    led_state |= 1<<0;
  }
  else{
    led_state &= ~1<<0;
  }
  set_leds();
}

void Arduino_Alvik::set_illuminator(const bool value){
  if (value){
    led_state |= 1<<1;
  }
  else{
    led_state &= ~1<<1;
  }
  set_leds();
}

void Arduino_Alvik::set_servo_positions(const uint8_t a_position, const uint8_t b_position){
  msg_size = packeter->packetC2B('S', a_position, b_position);
  uart->write(packeter->msg, msg_size);
}

void Arduino_Alvik::set_behaviour(const uint8_t behaviour){
  msg_size = packeter->packetC1B('B', behaviour);
  uart->write(packeter->msg, msg_size);
}

void Arduino_Alvik::get_version(uint8_t & upper, uint8_t & middle, uint8_t & lower){
  while (!xSemaphoreTake(version_semaphore, 5)){}
  upper = version[0];
  middle = version[1];
  lower = version[2];
  xSemaphoreGive(version_semaphore);
}


//-----------------------------------------------------------------------------------------------//
//                                       RGB led class                                           //
//-----------------------------------------------------------------------------------------------//

Arduino_Alvik::ArduinoAlvikRgbLed::ArduinoAlvikRgbLed(HardwareSerial * serial, ucPack * packeter, String label, 
                                                      uint8_t * led_state, uint8_t offset){
  _serial = serial;
  _packeter = packeter;
  this->label = label;
  _led_state = led_state;
  _offset = offset;
}

void Arduino_Alvik::ArduinoAlvikRgbLed::operator=(const ArduinoAlvikRgbLed& other){ 
  _serial = other._serial;
  _packeter = other._packeter;
  label = other.label;
  _led_state = other._led_state;
  _offset = other._offset;
  _msg_size = other._msg_size;

}

void Arduino_Alvik::ArduinoAlvikRgbLed::set_color(const bool red, const bool green, const bool blue){
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


//-----------------------------------------------------------------------------------------------//
//                                         wheel class                                           //
//-----------------------------------------------------------------------------------------------//

Arduino_Alvik::ArduinoAlvikWheel::ArduinoAlvikWheel(HardwareSerial * serial, ucPack * packeter, uint8_t label, 
                                                    float * joint_velocity, float * joint_position, float wheel_diameter){
  _serial = serial;
  _packeter = packeter;
  _label = label;
  _wheel_diameter = wheel_diameter;
  _joint_velocity = joint_velocity;
  _joint_position = joint_position;
}

void Arduino_Alvik::ArduinoAlvikWheel::reset(const float initial_position, const uint8_t unit){
  _msg_size = _packeter->packetC2B1F('W', _label, 'Z', convert_angle(initial_position, unit, DEG));
  _serial->write(_packeter->msg, _msg_size);
}

void Arduino_Alvik::ArduinoAlvikWheel::set_pid_gains(const float kp, const float ki, const float kd){
  _msg_size = _packeter->packetC1B3F('P', _label, kp, ki, kd);
  _serial->write(_packeter->msg, _msg_size);
}

void Arduino_Alvik::ArduinoAlvikWheel::stop(){
  set_speed(0);
}

void Arduino_Alvik::ArduinoAlvikWheel::set_speed(const float velocity, const uint8_t unit){
  if (unit==PERCENTAGE){
    converted_vel = (velocity/100.0)*MOTOR_MAX_RPM;
  }
  else{
    converted_vel = convert_rotational_speed(velocity, unit, RPM);
  }
  _msg_size = _packeter->packetC2B1F('W', _label, 'V', converted_vel);
  _serial->write(_packeter->msg, _msg_size);
}

float Arduino_Alvik::ArduinoAlvikWheel::get_speed(const uint8_t unit){
  if (unit==PERCENTAGE){
    return ((* _joint_velocity)/MOTOR_MAX_RPM)*100.0;
  }
  return convert_rotational_speed(* _joint_velocity, RPM, unit);
}

void Arduino_Alvik::ArduinoAlvikWheel::set_position(const float position, const uint8_t unit){
  _msg_size = _packeter->packetC2B1F('W', _label, 'P', convert_angle(position, unit, DEG));
  _serial->write(_packeter->msg, _msg_size);
}

float Arduino_Alvik::ArduinoAlvikWheel::get_position(const uint8_t unit){
  return convert_angle(* _joint_position, DEG, unit);
}



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
