/*
 * Copyright (c) 2024 PeanutKing Solution
 *
 * @file        PeanutKingDef.h
 * @summary     PeanutKing Solution Arduino Defines
 * @version     1.0
 * @author      Jack Kwok
 * @date        3 January 2024
 */


#ifndef PeanutKingDef_H
#define PeanutKingDef_H

#include <Arduino.h>

// All unused V2 legacy structs (motor_t, pulsein_t, pulseinColor_t,
// readpin_t, led_t, btData_t) removed in V4 rewrite.

const float pi = 3.1415926535897;

// Logical positions index
enum SensorPos : uint8_t {
  Front = 0,
  Right = 1,
  Back = 2,
  Left = 3,
  PositionCount = 4
};

#endif