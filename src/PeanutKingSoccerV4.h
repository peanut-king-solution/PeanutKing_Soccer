/*
 * Copyright (c) 2024 PeanutKing Solution
 *
 * @file        PeanutKingSoccerV4.h
 * @summary     Soccer Robot V3 Library
 * @version     3.4.0
 * @author      Jack Kwok
 * @date        2 January 2024
 * 
 * @log         3.3.0 - 5  Jun 2023
 *              3.1.0 - 26 Jul 2022
 */


#ifndef PeanutKing_Soccer_V4_H
#define PeanutKing_Soccer_V4_H

#include <pins_arduino.h>
#include "IICIT.h"
#include "PeanutKingDef.h"
#include "SlowSoftI2CMaster.h"
#include <SPI.h>				// must include this here (or else IDE can't find it)
                                           
#include <PDQ_GFX.h>				// PDQ: Core graphics library
#include <PDQ_ST7735.h>			// PDQ: Hardware-specific driver library
#include <pcint.h>
// #include <Fonts/FreeSerif12pt7b.h>	// include fancy serif font
// #include <Fonts/FreeSans12pt7b.h>	// include fancy sans-serif font
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


#define DEBUGMODE       1


// Soccer Sensorboard Register Address
#define  IR_RAW         0x0    // 2byte*12   (0x10 - 0x27)
#define  IR_MAX         0x11
#define  IR_MIN         0x12
#define  IR_ANGLE       0x13    // 2byte
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

#define TFT_CS  0 // TFT LCD的CS PIN腳
#define TFT_DC   53 // TFT DC(A0、RS) 
#define TFT_RST  50 // TFT Reset
#define TFT_SCL  52 // TFT Reset
#define TFT_SDA  51 // TFT Resets
// const uint8_t
//   STATERESET   = 0,
//   STATESET     = 1;

// motor,  + clockwise turn when positive value
// 1 4
// 2 3

// 1. userdefinebutton set of movement
// 2. attributes
typedef enum{
          // RGB
  LED_OFF,    // 000
  LED_BLUE,   // 001
  LED_GREEN,  // 010
  LED_CYAN,   // 011
  LED_RED,    // 100
  LED_PURPLE, // 101
  LED_YELLOW, // 110
  LED_WHITE   // 111
}obBrdLEDCL;
typedef enum{
  CL1,
  CL2,
  CL3,
  CL4,
  CL5,
  CL6,
  CL7,
  CL8
}CL_SENSOR;
typedef enum{
  U1,
  U2,
  U3,
  U4
}ULTR_SENSOR;

typedef enum{
  S1_P = 10,
  S2_P,
  S3_P,
  S4_P,
}S_PIN;

typedef enum{
  D6_P = 56,
  D5_P,
  D4_P,
  D3_P,
  D2_P,
  D1_P,
}D_PIN;

typedef enum{
  A4_P = 62,
  A3_P,
  A2_P,
  A1_P,
}A_PIN;
typedef enum {
  BLACK,  
  WHITE,   
  GREY,
  RED,      
  GREEN,   
  BLUE, 
  YELLOW,   
  CYAN     
}color_sensor_color;


class PeanutKingSoccerV4 {
 public:
  PeanutKingSoccerV4(void);

/* =============================================================================
 *                              Functions
 * ============================================================================= */

  // uint16_t sort(uint16_t a[], uint8_t size);
  // hsv_t    rgb2hsv(rgb_t in);

  // get sensor reading
  bool
    buttonRead(uint8_t),
    buttTrigRead(uint8_t),
    whiteLineCheck(uint8_t,uint16_t);
  uint8_t 
    floorColorReadRaw(uint8_t, uint8_t = 0),    
    compoundMaxEye(void),
    compoundMaxEyeVal(void),
    compoundEyeVal(uint8_t n),
    getColorSensor(uint8_t);
  uint16_t
    floorColorRead(uint8_t),
    ultrasonicRead(uint8_t),
    compassRead(void),
    whiteLineCal(uint8_t = 00);
  uint8_t* compoundEyeRead();
  void 
    compoundEyeCal(float* calData);
  uint16_t
    getRedColor(uint8_t i);

  void
    setOnBrdLED(uint8_t color),
    setOnBrdLED(uint8_t LED, uint8_t status),
    // bluetoothSend(char[]),
    // bluetoothReceive(void),
    bluetoothRemote(void),
    bluetoothAttributes(void);
  
  uint8_t colorReadAll(void);
  IICIT::status_t rxCpltCallback(const IICIT::status_t status);
  void 
    init(uint8_t = 0),
    // autoScanning(void),
    enableScanning(bool, uint16_t, bool),
    dataFetch(void),
    I2CSensorRead(IICIT::Handle handle, uint8_t sensor, uint8_t length),
    I2CSensorSend(IICIT::Handle handle, uint8_t sensor, uint8_t *data, uint8_t length),
    // I2CSend(int8_t addr, uint8_t *data, uint8_t length),
    // I2CRead(int8_t addr, uint8_t *data, uint8_t length),
    setColorBL(uint8_t r, uint8_t g, uint8_t b, uint8_t w),

    motorControl(float,float,float),
    motorSet(uint8_t, int16_t),
    move(int16_t, int16_t),
    moveSmart(uint16_t, int16_t, int16_t = 0, uint8_t = 5),
    motorStop(void);

  uint8_t motorTest (void);

  void Chase(int& direct, int& speed, int& rotation);
  void Back(int& direct, int& speed, int& rotation);
  hsl_t getColorSensorHSL(uint8_t color_sensor_num);
  rgb_t getColorSensorRGB(uint8_t color_sensor_num);

  /* Bottom Level Library */
  void
    motorDisable(void),
    buttons(void);



  // Constant  ======================================================================
  const int8_t 
    PAGEUPPERLIMIT = 6,
    PAGELOWERLIMIT = 0;
  const uint8_t
  sensorBoardAddr = 0x13,
  compass_address = 0x08;
  const uint8_t
    numLEDs     = 8;  // Number of RGB LEDs in strip

  // Pin Allocation ==================================================================
  const uint8_t
    buttonPin[4],
    ledPin[3],
    in1Pin[4],
    in2Pin[4],
    APin[4],
    DPin[6],
    pwmPin[4],
    ULTPin_trig[4];
  uint8_t  ULTPin_echo[4];
  SlowSoftI2CMaster  swiic[8];
  
  PDQ_ST7735 tft;

    // in1Pin      in2  inh1
    // 2, OCR3B,    3   4
    // 5, OCR3A,    6   7
    // 8, OCR4C,    9   10
    // 11,OCR1A,    12  13
    
  // Variables =======================================================================
  bool
    buttonPressed[3],
    isWhite[8]  = {false},
    onBound[8]  = {false},
    outBound[8] = {false};
  uint8_t 
    maxEye,
    groundColor[4],   //color sensor
    led[33],
    rxBuff[50],
    txBuff[50];
  uint16_t
    compass,
    eyeAngle;
    uint8_t eye[12];          // 12 ir reading
  uint16_t ultrasonic[4];    //4 ultrasonic reading
  uint16_t whiteLineThreshold[8] = {30, 30, 30, 30};

  buttonStatus_t button[3] = {NONE};

  int16_t currentSpeed[4] = {0,0,0,0};
  int16_t targetSpeed[4] = {0,0,0,0};

  rgb_t colorRGB[8];
  hsl_t colorHSL[8];
  hsv_t colorHSV[8];

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
    systemTime;      //a reference 100Hz clock, 0-100 every second
  uint32_t
    screenTicks = 0,
    sysTicks = 0;
    
  uint16_t tim1Count = 0;

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
  IICIT::Handle senbrdHandle;
  IICIT::Handle topbrdHandle;
  private:
  void ULT_Echo_dect(uint8_t);
  static void ULT_Echo_dect_0();
  static void ULT_Echo_dect_1();
  static void ULT_Echo_dect_2();
  static void ULT_Echo_dect_3();
  static void (*ULT_Echo_dect_ptr[4])();
  uint32_t ULT_dt[4];
  uint32_t ULT_get_interval;
  uint8_t ultra_send_seq = 0;

};

#endif



