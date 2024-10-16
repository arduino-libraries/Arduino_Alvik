/*
    This file is part of the Arduino_Alvik library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#include "Arduino_Alvik.h"
#include "unit_conversions.h"
#include "default_colors.h"

Arduino_Alvik::Arduino_Alvik():i2c(Wire){
  update_semaphore = xSemaphoreCreateMutex();
  uart = new HardwareSerial(UART); //&Serial0
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

  left_wheel = ArduinoAlvikWheel(uart, packeter, 'L', &joints_velocity[0], &joints_position[0], WHEEL_DIAMETER_MM, *this);
  right_wheel = ArduinoAlvikWheel(uart, packeter, 'R', &joints_velocity[1], &joints_position[1], WHEEL_DIAMETER_MM, *this);

  servo_A = ArduinoAlvikServo(uart, packeter, 'A', 0, servo_positions);
  servo_B = ArduinoAlvikServo(uart, packeter, 'B', 1, servo_positions);
}

void Arduino_Alvik::reset_hw(){                                                   //it is private
  digitalWrite(RESET_STM32, LOW);
  delay(100);
  digitalWrite(RESET_STM32, HIGH);
  delay(100);
}

void Arduino_Alvik::wait_for_ack(){
  waiting_ack = 0x00;
  while(last_ack != 0x00){
    delay(20);
  }
  waiting_ack = NO_ACK;
}

bool Arduino_Alvik::wait_for_fw_check(){
  while ((fw_version[0]==0)&&(fw_version[1]==0)&&(fw_version[2]==0)){
    delay(20);
  }
  if (check_firmware_compatibility()){
    return true;
  }
  else{
    return false;
  }
}

int Arduino_Alvik::begin(const bool verbose, const uint8_t core){  
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  Wire.begin();

  verbose_output = verbose;

  last_ack = NO_ACK;
  waiting_ack = NO_ACK;

  fw_version[0] = 0;
  fw_version[1] = 0;
  fw_version[2] = 0;

  led_state = 0;

  line_sensors[0] = 0;
  line_sensors[1] = 0;
  line_sensors[2] = 0;

  color_sensor[0] = 0;
  color_sensor[1] = 0;
  color_sensor[2] = 0;
  rgb_normalized[0] = 0.0;
  rgb_normalized[1] = 0.0;
  rgb_normalized[2] = 0.0;
  hsv[0] = 0.0;
  hsv[1] = 0.0;
  hsv[2] = 0.0;

  load_color_calibration();
  /*
  white_cal[0] = WHITE_CAL[0];
  white_cal[1] = WHITE_CAL[1];
  white_cal[2] = WHITE_CAL[2];
  black_cal[0] = BLACK_CAL[0];
  black_cal[1] = BLACK_CAL[1];
  black_cal[2] = BLACK_CAL[2];
  */

  servo_positions[0] = 90;
  servo_positions[1] = 90;

  orientation[0] = 0.0;
  orientation[1] = 0.0;
  orientation[2] = 0.0;

  move_bits = 0;

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

  battery = 0.0;
  battery_soc = 0.0;
  battery_is_charging = false;



  uart->begin(UART_BAUD_RATE);


  pinMode(CHECK_STM32, INPUT_PULLDOWN);
  pinMode(RESET_STM32, OUTPUT);
  pinMode(NANO_CHK, OUTPUT);
  digitalWrite(NANO_CHK, LOW);

  if (!is_on()){
    delay(1000);
    idle();
  }

  //begin_update_thread();
  xTaskCreatePinnedToCore(this->update_thread, "update", 10000, this, 1, &update_task, core);

  delay(100);
  reset_hw();

  uart->flush();
  while (uart->available()){
    uart->read();
  }

  wait_for_ack();
  if (!wait_for_fw_check()){
    if (verbose_output){
      Serial.println("\n********** PLEASE UPDATE ALVIK FIRMWARE (required: "+String(REQUIRED_FW_VER_UP)+"."+String(REQUIRED_FW_VER_MID)+"."+String(REQUIRED_FW_VER_LOW)+")! Check documentation **********\n");
      return -2;
    }
  }

  set_illuminator(true);
  set_behaviour(BEHAVIOUR_ILLUMINATOR_RISE);
  set_behaviour(BEHAVIOUR_BATTERY_ALERT);
  set_servo_positions(servo_positions[0],servo_positions[1]);

  return 0;
}

void Arduino_Alvik::stop(){
  set_wheels_speed(0,0);
}

bool Arduino_Alvik::is_on(){
  if (digitalRead(CHECK_STM32)==0){
    return false;
  }
  return true;
}

void Arduino_Alvik::idle(){
  digitalWrite(NANO_CHK, HIGH);
  delay(500);
  bool led_value=false;
  while(!is_on()){
    //read battery value
    battery = battery_measure();
    battery_is_charging = true;
    if (verbose_output){
      Serial.print(round(battery));
      Serial.println("%");
    }
    if (battery_soc>CHARGE_THRESHOLD){
      digitalWrite(LED_GREEN, LOW);
      digitalWrite(LED_RED, HIGH);
    }
    else{
      digitalWrite(LED_GREEN, HIGH);
      led_value = !led_value;
      digitalWrite(LED_RED, led_value);
    }
    delay(1000);
  }
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(NANO_CHK, LOW);
}


//-----------------------------------------------------------------------------------------------//
//                                          update                                               //
//-----------------------------------------------------------------------------------------------//

void Arduino_Alvik::update(const int delay_value){                                //it is private
  while (update_task != NULL){
    if (!is_on()){
      idle();
      reset_hw();
      uart->flush();
      delay(1000);
      wait_for_ack();
      set_illuminator(true);
      set_behaviour(BEHAVIOUR_ILLUMINATOR_RISE);
      set_behaviour(BEHAVIOUR_BATTERY_ALERT);
    }
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
      if (waiting_ack == NO_ACK){
        packeter->unpacketC1B(code, last_ack);
        last_ack = 0x00;
      } else {
        packeter->unpacketC1B(code, last_ack);
      }
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

    // get tilt and shake
    case 'm':
      packeter->unpacketC1B(code, move_bits);
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
    
    // get fw_version: Up, Mid, Low
    case 0x7E:
      while (!xSemaphoreTake(version_semaphore, 5)){}
      packeter->unpacketC3B(code, fw_version[0], fw_version[1], fw_version[2]);
      xSemaphoreGive(version_semaphore);
      break;

    // get battery parcentage: state of charge
    case 'p':
      packeter->unpacketC1F(code, battery);
      battery_is_charging = (battery > 0) ? true : false;
      battery = abs(battery);
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

void Arduino_Alvik::set_wheels_position(const float left, const float right, const uint8_t unit, const bool blocking){
  msg_size = packeter->packetC2F('A', convert_angle(left, unit, DEG), convert_angle(right, unit, DEG));
  uart->write(packeter->msg, msg_size);
  waiting_ack = 'P';
  if (blocking){
    wait_for_target(max(left, right) / MOTOR_CONTROL_DEG_S);
  }
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
    converted_angular = (angular/ROBOT_MAX_DEG_S)*100.0;
  }
  else{
    converted_angular = convert_rotational_speed(angular, angular_unit, DEG_S);
  }
  msg_size = packeter->packetC2F('V', convert_speed(linear, linear_unit, MM_S), converted_angular);
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
  if (waiting_ack == NO_ACK){
    return true;
  }
  if (last_ack == waiting_ack){
    msg_size = packeter->packetC1B('X', 'K');
    uart->write(packeter->msg, msg_size);
    waiting_ack = NO_ACK;
    last_ack = 0x00;
    delay(100);
    return true;
  }
  return false;
}

void Arduino_Alvik::wait_for_target(const int idle_time){                                             //it is private
  unsigned long start_t = millis();
  
  while (true){
    if (((millis() - start_t) >= idle_time*1000) && is_target_reached()) {
      break;
    } else
    {
      delay(100);
    }
    
  }
}

void Arduino_Alvik::rotate(const float angle, const uint8_t unit, const bool blocking){
  delay(200);
  msg_size = packeter->packetC1F('R', convert_angle(angle, unit, DEG));
  uart->write(packeter->msg, msg_size);
  waiting_ack = 'R';
  if (blocking){
    wait_for_target(round(angle/MOTOR_CONTROL_DEG_S));
  }
}

void Arduino_Alvik::move(const float distance, const uint8_t unit, const bool blocking){
  delay(200);
  msg_size = packeter->packetC1F('G', convert_distance(distance, unit, MM));
  uart->write(packeter->msg, msg_size);
  waiting_ack = 'M';
  if (blocking){
    wait_for_target(round(distance/MOTOR_CONTROL_MM_S));
  }
}

void Arduino_Alvik::brake(){
  drive(0,0);
}


//-----------------------------------------------------------------------------------------------//
//                                              sensors                                          //
//-----------------------------------------------------------------------------------------------//

float Arduino_Alvik::battery_measure(){                                             //it is private
  Wire.end();
  pinMode(A4,OUTPUT);
  pinMode(A5,OUTPUT);
  digitalWrite(A4,HIGH);
  digitalWrite(A5,HIGH);
  delay(100);
  digitalWrite(A4,LOW);
  digitalWrite(A5,LOW);
  Wire.begin();
  Wire.beginTransmission(0x36);
  Wire.write(0x06);
  if (Wire.endTransmission(false)!=0){
    Serial.println("Error on opening BMS");
    while(1);
  }
  else{
    Wire.requestFrom(0x36, 2);
    battery_v[0] = Wire.read();
    battery_v[1] = Wire.read();
    battery_val = (battery_v[1] << 8) + battery_v[0]; 
    battery_soc = battery_val * 0.00390625;
  }
  Wire.begin();
  return battery_soc;
}


void Arduino_Alvik::get_line_sensors(int & left, int & center, int & right){
  while (!xSemaphoreTake(line_semaphore, 5)){}
  left = line_sensors[0];
  center = line_sensors[1];
  right = line_sensors[2];
  xSemaphoreGive(line_semaphore);
}


void Arduino_Alvik::get_color_raw(int & red, int & green, int & blue){
  while (!xSemaphoreTake(color_semaphore, 5)){}
  red = color_sensor[0];
  green = color_sensor[1];
  blue = color_sensor[2];
  xSemaphoreGive(color_semaphore);
}

float Arduino_Alvik::limit(float value, const float min, const float max){                //it is private
  if (value < min){
    value = min;
  }
  if (value > max){
    value = max;
  }
  return value;
}

float Arduino_Alvik::normalize(float value, const float min, const float max){            //it is private
  return (value - min)/(max-min);
}

void Arduino_Alvik::rgb2norm(const int16_t r, const int16_t g, const int16_t b, float & r_norm, float & g_norm, float & b_norm){
  r_norm = limit(r, black_cal[0], white_cal[0]);
  r_norm = normalize(r_norm, black_cal[0], white_cal[0]);
  g_norm = limit(g, black_cal[1], white_cal[1]);
  g_norm = normalize(g_norm, black_cal[1], white_cal[1]);
  b_norm = limit(b, black_cal[2], white_cal[2]);
  b_norm = normalize(b_norm, black_cal[2], white_cal[2]);
}

void Arduino_Alvik::norm2hsv(const float r, const float g, const float b, float & h, float & s, float & v){
  float min = r;
  float max = r;
  if (g < min){
    min = g;
  }
  if (b < min){
    min = b;
  }
  if (g > max){
    max = g;
  }
  if (b > max){
    max = b;
  }

  v = max;

  float delta = max-min;
  
  if (delta < 0.00001){
    h = 0.0;
    s = 0.0;
    return;
  }

  if (max > 0.0){
    s = delta/max;
  }
  else{
    s = 0.0;
    h = -1;
    return;
  }

  if (r >= max){
    h = (g-b)/delta;
  }
  else{
    if (g >= max){
      h = 2.0+(b-r)/delta;
    }
    else{
      h = 4.0+(r-g)/delta;
    }
  }
  h *= 60.0;
  if (h < 0){
    h+=360.0;
  }
}

void Arduino_Alvik::get_color(float & value0, float & value1, float & value2, const uint8_t format){
  while (!xSemaphoreTake(color_semaphore, 5)){}
  rgb2norm(color_sensor[0], color_sensor[1], color_sensor[2], rgb_normalized[0], rgb_normalized[1], rgb_normalized[2]);
  xSemaphoreGive(color_semaphore);
  if (format == RGB){
    value0 = rgb_normalized[0];
    value1 = rgb_normalized[1];
    value2 = rgb_normalized[2];
  }
  else{
    if (format == HSV){
      norm2hsv(rgb_normalized[0], rgb_normalized[1], rgb_normalized[2], hsv[0], hsv[1], hsv[2]);
      value0 = hsv[0];
      value1 = hsv[1];
      value2 = hsv[2];
    }
    else{
      value0 = -1;
      value1 = -1;
      value2 = -1;
    }
  }
}

uint8_t Arduino_Alvik::get_color_id(const float h, const float s, const float v){
  if (s < MINIMUM_SATURATION){
    if (v < 0.05){
      return BLACK_ID;
    }
    else{
      if (v < GREY_VALUE){
        return GREY_ID;
      }
      else{
        if (v < LIGHT_GREY_VALUE){
          return LIGHT_GREY_ID;
        }
        else{
          return WHITE_ID;
        }
      }
    }
  }
  else{
    if (v > COLOR_VALUE){
      if ((h >= YELLOW_MIN) && (h < YELLOW_MAX)){
        return YELLOW_ID;
      }
      else{
        if ((h >= YELLOW_MAX) && (h < LIGHT_GREEN_MAX)){
          return LIGHT_GREEN_ID;
        }
        else{
          if ((h >= LIGHT_GREEN_MAX) && (h < GREEN_MAX)){
            return GREEN_ID;
          }
          else{
            if ((h >= GREEN_MAX) && (h < LIGHT_BLUE_MAX)){
              return LIGHT_BLUE_ID;
            }
            else{
              if ((h >= LIGHT_BLUE_MAX) && (h < BLUE_MAX)){
                return BLUE_ID;
              }
              else{
                if ((h >= BLUE_MAX) && (h < VIOLET_MAX)){
                  return VIOLET_ID;
                }
                else{
                  if ((v < BROWN_MAX_VALUE) && (v < BROWN_MAX_SATURATION)){
                    return BROWN_ID;
                  }
                  else{
                    if (v > ORANGE_MIN_VALUE){
                      return ORANGE_ID;
                    }
                    else{
                      return RED_ID;
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
    else{
      return BLACK_ID;
    }
  }
}

uint8_t Arduino_Alvik::get_color_id(){
  float h,s,v;
  get_color(h, s, v, HSV);
  return get_color_id(h, s, v);
}

String Arduino_Alvik::get_color_label(const float h, const float s, const float v){
  return COLOR_LABELS[get_color_id(h, s, v)];
}

String Arduino_Alvik::get_color_label(){
  return COLOR_LABELS[get_color_id()];
}

void Arduino_Alvik::color_calibration(const uint8_t background){
  if ((background != BLACK_ID)&&(background != WHITE_ID)){
    return;
  }
  int red_avg = 0;
  int green_avg = 0;
  int blue_avg = 0;
  int red, green, blue;

  for (int i=0; i<CALIBRATION_ITERATIONS; i++){
    get_color_raw(red, green, blue);
    red_avg += red;
    green_avg += green;
    blue_avg += blue;
    delay(10);
  }

  red_avg = red_avg/float(CALIBRATION_ITERATIONS);
  green_avg = green_avg/float(CALIBRATION_ITERATIONS);
  blue_avg = blue_avg/float(CALIBRATION_ITERATIONS);

  EEPROM.begin(COLOR_SIZE);
  if (background == WHITE_ID){
    EEPROM.writeUShort(WHITE_OFFSET, (int16_t)red_avg);
    EEPROM.writeUShort(WHITE_OFFSET+2, (int16_t)green_avg);
    EEPROM.writeUShort(WHITE_OFFSET+4, (int16_t)blue_avg);
  }
  else{
    EEPROM.writeUShort(BLACK_OFFSET, (int16_t)red_avg);
    EEPROM.writeUShort(BLACK_OFFSET+2,(int16_t) green_avg);
    EEPROM.writeUShort(BLACK_OFFSET+4, (int16_t)blue_avg);
  }
  EEPROM.end();
  load_color_calibration();
}

void Arduino_Alvik::load_color_calibration(){
  EEPROM.begin(COLOR_SIZE);
  if ((EEPROM.readUShort(WHITE_OFFSET)!=0)&&(EEPROM.readUShort(WHITE_OFFSET+2)!=0)&&(EEPROM.readUShort(WHITE_OFFSET+4)!=0)){
    white_cal[0] = EEPROM.readUShort(WHITE_OFFSET);
    white_cal[1] = EEPROM.readUShort(WHITE_OFFSET+2);
    white_cal[2] = EEPROM.readUShort(WHITE_OFFSET+4);
    black_cal[0] = EEPROM.readUShort(BLACK_OFFSET);
    black_cal[1] = EEPROM.readUShort(BLACK_OFFSET+2);
    black_cal[2] = EEPROM.readUShort(BLACK_OFFSET+4);
  }
  else{
    white_cal[0] = WHITE_DEFAULT_RED;
    white_cal[1] = WHITE_DEFAULT_GREEN;
    white_cal[2] = WHITE_DEFAULT_BLUE;
    black_cal[0] = BLACK_DEFAULT_RED;
    black_cal[1] = BLACK_DEFAULT_GREEN ;
    black_cal[2] = BLACK_DEFAULT_BLUE;
  }
  EEPROM.end();
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

bool Arduino_Alvik::get_shake(){
  return move_bits & 0b00000001;
}

String Arduino_Alvik::get_tilt(){
  if (move_bits & 0b00000100){
    return "X";
  }
  if (move_bits & 0b00001000){
    return "-X";
  }
  if (move_bits & 0b00010000){
    return "Y";
  }
  if (move_bits & 0b00100000){
    return "-Y";
  }
  if (move_bits & 0b01000000){
    return "Z";
  }
  if (move_bits & 0b10000000){
    return "-Z";
  }
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

float Arduino_Alvik::get_distance_top(const uint8_t unit){
  return convert_distance(distances[5], MM, unit);
}

float Arduino_Alvik::get_distance_bottom(const uint8_t unit){
  return convert_distance(distances[6], MM, unit);
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


int Arduino_Alvik::get_battery_charge(){
  return round(battery);
}

bool Arduino_Alvik::is_battery_charging(){
  return battery_is_charging;
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
  servo_positions[0] = a_position;
  servo_positions[1] = b_position;
  msg_size = packeter->packetC2B('S', a_position, b_position);
  uart->write(packeter->msg, msg_size);
}

void Arduino_Alvik::get_servo_positions(int & a_position, int & b_position){
  a_position = servo_positions[0];
  b_position = servo_positions[1];
}

void Arduino_Alvik::set_behaviour(const uint8_t behaviour){
  msg_size = packeter->packetC1B('B', behaviour);
  uart->write(packeter->msg, msg_size);
}

void Arduino_Alvik::get_version(uint8_t & upper, uint8_t & middle, uint8_t & lower, String version){
  if ((version=="fw")||(version=="FW")||(version=="firmware")){
    get_fw_version(upper,middle,lower);
  }
  else{
    if ((version=="lib")||(version=="LIB")){
      get_lib_version(upper,middle,lower);
    }
    else{
      upper = 0;
      middle = 0;
      lower = 0;
    }
  }
}

void Arduino_Alvik::get_fw_version(uint8_t & upper, uint8_t & middle, uint8_t & lower){
  while (!xSemaphoreTake(version_semaphore, 5)){}
  upper = fw_version[0];
  middle = fw_version[1];
  lower = fw_version[2];
  xSemaphoreGive(version_semaphore);
}

void Arduino_Alvik::get_lib_version(uint8_t & upper, uint8_t & middle, uint8_t & lower){
  upper = LIB_VER_UP;
  middle = LIB_VER_MID;
  lower = LIB_VER_LOW;
}

void Arduino_Alvik::get_required_fw_version(uint8_t & upper, uint8_t & middle, uint8_t & lower){
  upper = REQUIRED_FW_VER_UP;
  middle = REQUIRED_FW_VER_MID;
  lower = REQUIRED_FW_VER_LOW;
}

bool Arduino_Alvik::check_firmware_compatibility(){
  if ((fw_version[0]==REQUIRED_FW_VER_UP)&&(fw_version[1]==REQUIRED_FW_VER_MID)&&(fw_version[2]==REQUIRED_FW_VER_LOW)){
    return true;
  }
  return false;
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
                                                    float * joint_velocity, float * joint_position, float wheel_diameter, Arduino_Alvik & alvik):_alvik(&alvik){
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

void Arduino_Alvik::ArduinoAlvikWheel::set_position(const float position, const uint8_t unit, const bool blocking){
  _msg_size = _packeter->packetC2B1F('W', _label, 'P', convert_angle(position, unit, DEG));
  _serial->write(_packeter->msg, _msg_size);
  _alvik->waiting_ack = 'P';
  if (blocking){
    _alvik->wait_for_target(position / MOTOR_CONTROL_DEG_S);
  }
}

float Arduino_Alvik::ArduinoAlvikWheel::get_position(const uint8_t unit){
  return convert_angle(* _joint_position, DEG, unit);
}

//-----------------------------------------------------------------------------------------------//
//                                         servo class                                           //
//-----------------------------------------------------------------------------------------------//

Arduino_Alvik::ArduinoAlvikServo::ArduinoAlvikServo(HardwareSerial * serial, ucPack * packeter, char label,
                                                                    uint8_t servo_id, uint8_t * positions){
  _serial = serial;
  _packeter = packeter;
  _label = label;
  _servo_id = servo_id;
  _positions = positions;
}

void Arduino_Alvik::ArduinoAlvikServo::set_position(const uint8_t position){
  _positions[_servo_id] = position;
  _msg_size = _packeter->packetC2B('S', _positions[0], _positions[1]);
  _serial->write(_packeter->msg, _msg_size);
}

int Arduino_Alvik::ArduinoAlvikServo::get_position(){
  return _positions[_servo_id];
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
