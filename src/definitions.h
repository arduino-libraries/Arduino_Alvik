/*
    This file is part of the Arduino_Alvik library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include "Arduino.h"



#define LIB_VER_UP 1
#define LIB_VER_MID 1
#define LIB_VER_LOW 0

#define REQUIRED_FW_VER_UP  1
#define REQUIRED_FW_VER_MID 1
#define REQUIRED_FW_VER_LOW 0


#define RUN_ON_CORE_0 0
#define RUN_ON_CORE_1 1
#define NO_VERBOSE 0
#define VERBOSE 1

#define CHECK_STM32 A6
#define BOOT_STM32 D2
#define RESET_STM32 D3
#define NANO_CHK D4
#define UART 0
#define UART_BAUD_RATE 460800

#define DL1 left_led
#define DL2 right_led

#define CHARGE_THRESHOLD 97

#define NO_ACK 0xFF

const float WHEEL_DIAMETER_MM = 34.0;
const float WHEEL_TRACK_MM = 89.0;
const float MOTOR_MAX_RPM = 70.0;
const float ROBOT_MAX_DEG_S = 6*(2*MOTOR_MAX_RPM*WHEEL_DIAMETER_MM)/WHEEL_TRACK_MM;
const float MOTOR_CONTROL_MM_S = 100.0;
const float MOTOR_CONTROL_DEG_S = 100.0;

// unit conversion constants

#define CM 0
#define MM 1
#define M 2
#define IN 3
#define INCH 3
const float DISTANCE_UNITS[4] = {1.0, 0.1, 100.0, 2.54};


#define CM_S 0
#define MM_S 1
#define M_S 2
#define IN_S 3
#define INCH_S 3
const float SPEED_UNITS[4] = {1.0, 0.1, 100.0, 2.54};


#define DEG 0
#define RAD 1
#define REV 2
#define REVOLUTIONS 2
#define PERC 3
#define PERCENTAGE 3
const float ANGLE_UNITS[4] = {1.0, 57.2957795131, 360.0, 3.6};


#define RPM 0
#define DEG_S 1
#define RAD_S 2
#define REV_S 3
#define REVOLUTIONS_S 3
#define PERC 4
#define PERCENTAGE 4
const float ROTATIONAL_SPEED_UNITS[4] = {1.0, 0.16666666666, 9.54929658551, 60.0};


// behaviours
#define BEHAVIOUR_DISABLED 0
#define BEHAVIOUR_ILLUMINATOR_RISE 1
#define BEHAVIOUR_BATTERY_ALERT 2


// colors
#define RGB 0
#define HSV 1


#endif