/*
 * Copyright (c) 2020 PeanutKing Solution
 *
 * @file        PeanutKing_Soccer.h
 * @summary     Soccer Robot Library
 * @version     1.0
 * @author      Jack Kwok
 * @data        1 August 2020
 */


#ifndef PeanutKing_Soccer_H
#define PeanutKing_Soccer_H

// const uint16_t
//   ALLIOs       = 0xffff,
//   ALLSENSORS   = 0x0fff,
//   COMPASS      = 0x0001,    // 0.27ms
//   COMPOUNDEYE  = 0x0002,    // 1.45ms
//   ULTRASONIC   = 0x00f0,    // ~14ms ~ 32ms for 4
//   ULTRASONIC0  = 0x0010,    
//   ULTRASONIC1  = 0x0020,    
//   ULTRASONIC2  = 0x0040,    
//   ULTRASONIC3  = 0x0080,    
//   COLORSENSOR  = 0x0f00,    // ~10ms for 4 (2.7 FOR 4), new 7.6ms for 4
//   COLORSENSOR0 = 0x0100,    
//   COLORSENSOR1 = 0x0200,    
//   COLORSENSOR2 = 0x0400,    
//   COLORSENSOR3 = 0x0800,    
//   ALLOUTPUTs   = 0xf000,
//   LED          = 0x1000,    // ~0.5ms for 8 leds
//   MOTOR        = 0x2000,    // 0.42ms for motorcontrol (4motors)
//   LCDSCREEN    = 0x4000;

// typedef enum {
//   testLED = 1,
//   testMotor,
//   testCompass,
//   testUltrasonic,
//   testCompoundeye,
//   testColor,
//   testBT,
  
//   testAll = 99,
// } testUnit;

// typedef enum {
//   chaseball,
//   goal,
//   gohome,
//   gohome2,
//   goleft,
//   goright,
//   gofront
// } pressureTestStatus;

// typedef enum {
//   Idle = 0,
//   Joystick = 1,
//   PadButton,
//   ButtonDef,
//   Attributes,
//   EndOfData = 26,
//   DemoMode = 25,
// } btDataType;

class PeanutKing_Soccer {
 public:
  // Constructor 
  PeanutKing_Soccer(void);

/* =============================================================================
 *                              Functions
 * ============================================================================= */

//protected:

  // Constant  ===========================================================
};

#endif


