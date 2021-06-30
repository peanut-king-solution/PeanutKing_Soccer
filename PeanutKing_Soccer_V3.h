/*
 * Copyright (c) 2020 PeanutKing Solution
 *
 * @file        PeanutKing_Soccer_V3.h
 * @summary     Soccer Robot Library
 * @version     1.0
 * @author      Jack Kwok
 * @data        1 August 2020
 */

#include "PeanutKing_Soccer.h"

#ifndef PeanutKing_Soccer_V3_H
#define PeanutKing_Soccer_V3_H

// 1. userdefinebutton set of movement
// 2. attributes

typedef enum {
//  IRsensor    (0x10 - 0x2f) 29 bytes
  IR_RAW      = 0x10,   // 2byte*12
  IR_MAX      = 0x28,
  IR_MIN      = 0x29,
  IR_ANGLE    = 0x2a,
  IR_LIMIT    = 0x2c,
  
//  Ultrasonic  (0x30 - 0x37) 8 bytes
  ULT_DATA    = 0x30,   // 2byte*4

//  Compass     (0x40 - 0x5f) 21 + 6 bytes
  ACC_RAW     = 0x40,   // 6  (0-5)
  GYR_RAW     = 0x46,   // 6  (6-b)
  MAG_RAW     = 0x4c,   // 6  (c-2)
  
  GET_ROLL    = 0x54,   // 2byte
  GET_YAW     = 0x56,   // 2byte
  GET_PITCH   = 0x58,   // 2byte
  
  MAG_CENT    = 0x5a,   // xxyyzz
  
  GET_COMPASS = 0x55,   // 3byte
  
//  ColorSensor (0x60 - 0x8f) 44 bytes
  COLOR_RGB   = 0x50,   // 3byte*4
  COLOR_DEC   = 0x5f,   // 1byte*4
  COLOR_RAW   = 0x60,   // 8byte*4
  COLOR_HSL   = 0x80,   // 4byte*4
  COLOR_HSV   = 0x90,   // 4byte*4
  
//  Laser       (0x90 - 0x9f) 8 bytes
  LASER       = 0x90,   // 2byte*4
  
//  LED         (0xa0 - 0xdf) 64 bytes
  LED_RGB     = 0xa0,   // 4byte*8 32
  LED_HSV     = 0x80,   // 4byte*8 32
  
//  Motor       (0xf0 - 0xff)
  MOTOR_CTRL  = 0xf0,   // 4 motors * 2byte
  ENCODER     = 0xf8,   // 4 encoder* byte
} soccerRegAddr;

typedef enum {
  NONE  = 0,
  TAP   = 1,      // - ??
  PRESS = 2,      // --
  HOLD  = 3,      // ----
  TAP2  = 4,      // - -
  TAP3  = 5,      // - - -
  RELEASE = 6,
  RELEASE_S = 7,
  RELEASE_L = 8,
  
} button_t;

class PeanutKing_Soccer_V3: public PeanutKing_Soccer {
 public:
  PeanutKing_Soccer_V3(void);

  // functions =======================================================================
  // get sensor reading
  bool
    buttonRead(uint8_t),
    whiteLineCheck(uint8_t);
  uint8_t 
    floorColorReadRaw(uint8_t, uint8_t = black),
    floorColorRead(uint8_t);
  uint16_t
    compoundEyeRead(uint8_t = 13),
    ultrasonicRead(uint8_t),
    compassRead(void);
  uint16_t whiteLineCal(uint16_t, uint8_t);

  void 
    init(uint8_t = 0),
    autoScanning(void),

    setLED(uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w),
    dataFetch(void),
    I2CSensorRead(int8_t addr, soccerRegAddr sensor, uint8_t length),
    I2CSensorSend(int8_t addr, soccerRegAddr sensor, uint8_t *data, uint8_t length),
    I2CSend(int8_t addr, uint8_t *data, uint8_t length),
    I2CRead(int8_t addr, uint8_t *data, uint8_t length),

    motorControl(float,float,float),
    motorSet(uint8_t, int16_t),
    move(int16_t, int16_t),
    moveSmart(uint16_t, int16_t, int16_t = 0, uint8_t = 5),
    motorStop(void),

    buttons(void);

  uint8_t motorTest (void);
  
  void Chase(int& direct, int& speed, int& rotation);
  void Back(int& direct, int& speed, int& rotation);

  // Pin Allocation ==================================================================
  const uint8_t
    sensorBoardAddr,
    topBoardAddr,
    actledPin,
    buttonPin[3],
    pwmPin[4],
    dirPin[4],
    dir2Pin[4],
    diagPin[4];
    
  const uint8_t
    numLEDs     = 8;  // Number of RGB LEDs in strip
    
  // Variables =======================================================================
  bool
    button[3],
    buttonPressed[3],
    isWhite[4]  = {false},
    onBound[4]  = {false},
    outBound[4] = {false};
  uint8_t 
    maxEye,
    groundColor[4];   //color sensor
  uint16_t
    compass,
    eyeAngle,
    eye[13];          // 12 ir reading
  int16_t
    ultrasonic[4];    //4 ultrasonic reading
  rgb_t
    colorRGB[4];
  hsl_t
    colorHSL[4];
  hsv_t
    colorHSV[4];

  uint16_t
    whiteLineThreshold = 600;

  uint8_t led[32];

  uint8_t rxBuff[50];
  uint8_t txBuff[50];
};

#endif

