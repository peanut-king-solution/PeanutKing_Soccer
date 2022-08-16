/*
 * Copyright (c) 2022 PeanutKing Solution
 *
 * @file        PeanutKingSoccerV3.cpp
 * @summary     Soccer Robot V3 Library
 * @version     3.1
 * @author      Jack Kwok
 * @data        26 July 2022
 */

#include "IICIT.h"

#ifndef PeanutKing_Soccer_V3_H
#define PeanutKing_Soccer_V3_H


#define LED1   0x01
#define LED2   0x02
#define LED3   0x04
#define LED4   0x08
#define LED5   0x10
#define LED6   0x20
#define LED7   0x40
#define LED8   0x80

#define DEC 10
#define HEX 16
#define OCT 8
#ifdef  BIN // Prevent warnings if BIN is previously defined in "iotnx4.h" or similar
  #undef  BIN
#endif
#define BIN 2

// flags for backlight control
#define LCD_BACKLIGHT     0x08    // B00001000
#define LCD_NOBACKLIGHT   0x00    // B00000000

#define En B00000100  // Enable bit
//#define Rw B00000010  // Read/Write bit
#define Rs B00000001  // Register select bit

#define LCD_RETURNHOME    0x02
#define LCD_ENTRYLEFT  
#define LCD_ENTRYSHIFTDECREMENT 0x00
#define LCD_CLEARDISPLAY  0x01
#define LCD_SETDDRAMADDR  0x80

#define DEBUGMODE       1


// Soccer Sensorboard Register Address
#define  IR_RAW         0x10    // 2byte*12   (0x10 - 0x27)
#define  IR_MAX         0x28
#define  IR_MIN         0x29
#define  IR_ANGLE       0x2a    // 2byte
//  IR_LIMIT        0x2c
#define  IR_COUNT       0x2d    // 2byte
#define  IR_LEDEN       0x2f
#define  IR_CAL         0x30    // 2byte*12   (0x30 - 0x47)

#define  COLOR_RGB      0x50    // 3byte*4    (0x50 - 0x5b)
#define  COLOR_DEC      0x5c    // 1byte*4    (0x5c - 0x5f)
#define  COLOR_RAW      0x60    // 2byte*4*4  (0x60 - 0x7f)
#define  COLOR_HSL      0x80    // 4byte*4    (0x80 - 0x8f)
#define  COLOR_HSV      0x90    // 4byte*4    (0x90 - 0x9f)
#define  COLOR_BL       0xa0    // 4byte*4    (0xa0 - 0xaf)

#define  IR_ARR_MAX     0xb0    // 2byte*12   (0xb0 - 0xc7)
#define  IR_ARR_MIN     0xc8    // 2byte*12   (0xc8 - 0xdf)


// Soccer Topboard Register Address
#define  ULT_DATA       0x30    // 2byte*4    (0x30 - 0x37)
#define  LED_RGB        0xa0    // 4byte*8 32 (0xa0 - 0xbf)
#define  LED_HSV        0xc0    // 4byte*8 32 (0xc0 - 0xdf)

//#define  MOTOR           0xf0    // 4 motors * 2byte
//#define  ENCODER         0xf8    // 4 encoder* byte

// Compass Module Register Address
#define  ACC_RAW        0x40    // 6byte   (0-5)
#define  GYR_RAW        0x46    // 6byte  (6-b)
#define  MAG_RAW        0x4c    // 6byte  (c-2)

#define  GET_COMPASS    0x55    // 3byte
#define  GET_ROLL       0x54    // 2byte
#define  GET_YAW        0x56    // 2byte
#define  GET_PITCH      0x58    // 2byte
#define  MAG_CENT       0x5a    // xxyyzz




const float pi = 3.1415926535897;

typedef struct {
  uint16_t r;
  uint16_t g;
  uint16_t b;
} rgb_t;

typedef struct {
  uint16_t h;
  uint8_t s;
  uint8_t v;
} hsv_t;

typedef struct {
  uint16_t h;
  uint8_t s;
  uint8_t l;
} hsl_t;

// typedef struct {
//   volatile uint8_t *port;
//   uint8_t mask;
//   uint8_t numLEDs;
//   uint8_t numBytes;
//   uint8_t *pixels;     // Holds LED color values (3 or 4 bytes each)
// } led_t;

typedef enum { front = 0, left, right, back } sensorNum_t;

typedef enum {
  black=0,  white,   grey,
  red,      green,   blue, 
  yellow,   cyan,    magenta
} color;

// const uint8_t
//   STATERESET   = 0,
//   STATESET     = 1;

// const float pi = 3.1415926535897;

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


// motor,  + clockwise turn when positive value
// 1 4
// 2 3

// 1. userdefinebutton set of movement
// 2. attributes


#define TAP_DURATION    140
#define HOLD_DURATION   1000
#define WAIT_DURATION   130

typedef enum {
  NONE  = 0,
  TAP   = 1,      // -
  PRESS = 2,      // ---
  HOLD  = 3,      // ------
  TAP2  = 4,      // - -
  TAP3  = 5,      // - - -
  RELEASE = 6,
  RELEASE_S = 7,
  RELEASE_L = 8,

  TAP1_W = 10,
  TAP2_W = 11,
  TAP3_W = 12,
  HOLD2  = 13,    // - --
  
  TAP2_R = 16,
  TAP3_R = 17,
} buttonStatus_t;

// typedef struct {
//   uint32_t holdTimer;
//   uint16_t button;
//   uint16_t b;
// } button_t;



class PeanutKingSoccerV3 {
 public:
  PeanutKingSoccerV3(void);

/* =============================================================================
 *                              Functions
 * ============================================================================= */

  // uint16_t sort(uint16_t a[], uint8_t size);
  // hsv_t    rgb2hsv(rgb_t in);

  // get sensor reading
  bool
    buttonRead(uint8_t),
    buttTrigRead(uint8_t),
    whiteLineCheck(uint8_t);
  uint8_t 
    floorColorReadRaw(uint8_t, uint8_t = black),
    floorColorRead(uint8_t);
  uint16_t
    compoundEyeRead(uint8_t = 13),
    ultrasonicRead(uint8_t),
    compassRead(void),
    whiteLineCal(uint16_t = 20, uint8_t = 0);

  void compoundEyeCal(float* calData);
  void
    actLED(bool),
    lcdMenu(void),
    bluetoothSend(char[]),
    bluetoothReceive(void),
    bluetoothRemote(void),
    bluetoothAttributes(void);
  
  uint8_t colorReadAll(void);
  IICIT::status_t LCDCallback(const IICIT::status_t status);

  void 
    init(uint8_t = 0),
    // autoScanning(void),
    enableScanning(bool, uint16_t, bool),

    setLED(uint8_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w),
    dataFetch(void),
    I2CSensorRead(IICIT::Handle handle, uint8_t sensor, uint8_t length),
    I2CSensorSend(IICIT::Handle handle, uint8_t sensor, uint8_t *data, uint8_t length),
    I2CSend(int8_t addr, uint8_t *data, uint8_t length),
    I2CRead(int8_t addr, uint8_t *data, uint8_t length),
    setColorBL(uint8_t r, uint8_t g, uint8_t b, uint8_t w),

    motorControl(float,float,float),
    motorSet(uint8_t, int16_t),
    move(int16_t, int16_t),
    moveSmart(uint16_t, int16_t, int16_t = 0, uint8_t = 5),
    motorStop(void),
    buttons(void);

  uint8_t buttons(uint8_t);
  uint8_t motorTest (void);

  void Chase(int& direct, int& speed, int& rotation);
  void Back(int& direct, int& speed, int& rotation);


  /* LCD Library */
  void
    printSpace(uint32_t, uint8_t digits = 3),
    setScreen(uint8_t, uint8_t, char[] ),
    setScreen(uint8_t, uint8_t, int16_t, uint8_t digits = 3),
    lcdSetup(void),
    lcdClear(void),
    setCursor(uint8_t col, uint8_t row);
  void
    send(uint8_t value, uint8_t mode),
    write4bits(uint8_t value),
    expanderWrite(uint8_t _data);
  size_t 
    printNumber(unsigned long, uint8_t),
    print(long, int = DEC),
    print(const char *),
    write(uint8_t);

  // Constant  ======================================================================
  const int8_t 
    PAGEUPPERLIMIT = 6,
    PAGELOWERLIMIT = 0;
  
  const uint8_t
    compass_address = 0x08,
    LCD_Addr        = 0x20;

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
    buttonPressed[3],
    isWhite[4]  = {false},
    onBound[4]  = {false},
    outBound[4] = {false};
  uint8_t 
    maxEye,
    groundColor[4],   //color sensor
    led[33],
    rxBuff[50],
    txBuff[50];
  uint16_t
    compass,
    eyeAngle,
    eye[13];          // 12 ir reading
  int16_t ultrasonic[4];    //4 ultrasonic reading
  uint16_t whiteLineThreshold[4] = {30, 30, 30, 30};

  buttonStatus_t button[3] = {NONE};

  rgb_t colorRGB[4];
  hsl_t colorHSL[4];
  hsv_t colorHSV[4];

  // Sensor Reading ======================================================
  bool
    btButton[10],
    // autoScanEnabled   = true,
    motorEnabled      = true,
    motorBrakeEnabled = true,
    ledEnabled        = false,
    ledFlashEnabled   = false;
  uint16_t
    EYEBOUNDARY = 20,
    LCD_backlightval,
    systemTime;      //a reference 100Hz clock, 0-100 every second
  uint32_t
    screenTicks = 0,
    sysTicks = 0;
    
  // BT Variables ========================================================
  uint8_t
    btButtonIndex,
    btGestureCode,
    btButtonFunction[4],
    btAttributes[5]  = {5,5,5,5,5},
    btTxBuffer[50],
    btRxBuffer[50]; //store at most the most updated 100 values from BT
  int16_t
    btDegree = 0,
    btDistance = 0,
    btRotate = 0;

  IICIT::Handle compssHandle;
  IICIT::Handle lcdScrHandle;
  IICIT::Handle senbrdHandle;
  IICIT::Handle topbrdHandle;
};

#endif



