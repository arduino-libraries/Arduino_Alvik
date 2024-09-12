/*
    This file is part of the Arduino_Alvik library.

    Copyright (c) 2024 Arduino SA

    This Source Code Form is subject to the terms of the Mozilla Public
    License, v. 2.0. If a copy of the MPL was not distributed with this
    file, You can obtain one at http://mozilla.org/MPL/2.0/.
    
*/


#ifndef __DEFAULT_COLORS_H__
#define __DEFAULT_COLORS_H__

#include "Arduino.h"

#define WHITE_DEFAULT_RED 450
#define WHITE_DEFAULT_GREEN 500
#define WHITE_DEFAULT_BLUE 510
#define BLACK_DEFAULT_RED 160
#define BLACK_DEFAULT_GREEN 200
#define BLACK_DEFAULT_BLUE 190
//int16_t WHITE_CAL[3] = {444, 342, 345};
//int16_t BLACK_CAL[3] = {153, 135, 123};


#define WHITE_OFFSET 0
#define BLACK_OFFSET 6
#define COLOR_SIZE 20

#define CALIBRATION_ITERATIONS 100


#define MINIMUM_SATURATION 0.1
#define BLACK_VALUE 0.05
#define GREY_VALUE 0.15
#define LIGHT_GREY_VALUE 0.8
#define COLOR_VALUE 0.1
#define YELLOW_MIN 20
#define YELLOW_MAX 90
#define LIGHT_GREEN_MAX 140
#define GREEN_MAX 170
#define LIGHT_BLUE_MAX 210
#define BLUE_MAX 250
#define VIOLET_MAX 280
#define BROWN_MAX_VALUE 0.5
#define BROWN_MAX_SATURATION 0.45
#define ORANGE_MIN_VALUE 0.77



#define BLACK_ID 0
#define GREY_ID 1
#define LIGHT_GREY_ID 2
#define WHITE_ID 3
#define YELLOW_ID 4
#define LIGHT_GREEN_ID 5
#define GREEN_ID 6
#define LIGHT_BLUE_ID 7
#define BLUE_ID 8
#define VIOLET_ID 9
#define BROWN_ID 10
#define ORANGE_ID 11
#define RED_ID 12







#endif