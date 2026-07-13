/*
 * Copyright (c) 2024 PeanutKing Solution
 *
 * @file        PeanutKingSoccerV4.h
 * @summary     Soccer Robot V4 Library
 * @version     4.0.0
 * @author      Jack Kwok
 * @date        2 January 2024
 *
 * @log         4.0.0 - 9  Jul 2024 - Extract Compass module
 *              3.3.0 - 5  Jun 2023
 *              3.1.0 - 26 Jul 2022
 */

#ifndef PeanutKing_Soccer_V4_H
#define PeanutKing_Soccer_V4_H

#include "PeanutKingDef.h"

#include "IICIT.h"
#include "SlowSoftI2CMaster.h"

#include "utils/Converter.h"
#include "utils/PIDController.h"
#include "modules/Motor/Motor.h"
#include "modules/Movement/Movement.h"
#include "modules/Compass/Compass.h"
#include "modules/ButtonManager/ButtonManager.h"
#include "modules/LedController/LedController.h"

#include <SPI.h>                   // must include this here (or else IDE can't find it)
#include <pcint.h>                 // Pin Change Interrupt Library
#include <PDQ_GFX.h>               // PDQ: Core graphics library
#include <PDQ_ST7735.h>            // PDQ: Hardware-specific driver library
#include <pins_arduino.h>          // Arduino pin definitions

// =============================================================================
//                              Macro Definitions
// =============================================================================

// Number Base
#define DEC 10
#define HEX 16
#define OCT 8
#ifdef  BIN // Prevent warnings if BIN is previously defined in "iotnx4.h" or similar
  #undef  BIN
#endif
#define BIN 2

// Debug Mode
#define DEBUGMODE       1

// Soccer Sensorboard Register Address
#define  IR_RAW         0x0    // 2byte*12   (0x10 - 0x27)
#define  IR_MAX         0x11
#define  IR_MIN         0x12
#define  IR_ANGLE       0x13   // 2byte
// IR_LIMIT 0x2c
#define  IR_COUNT       0x2d   // 2byte
#define  IR_LEDEN       0x2f
#define  IR_CAL         0x30   // 2byte*12   (0x30 - 0x47)

#define  COLOR_RGB      0x50   // 3byte*4    (0x50 - 0x5b)
#define  COLOR_DEC      0x5c   // 1byte*4    (0x5c - 0x5f)
#define  COLOR_RAW      0x60   // 2byte*4*4  (0x60 - 0x7f)
#define  COLOR_HSL      0x80   // 4byte*4    (0x80 - 0x8f)
#define  COLOR_HSV      0x90   // 4byte*4    (0x90 - 0x9f)
#define  COLOR_BL       0xa0   // 4byte*4    (0xa0 - 0xaf)

#define  IR_ARR_MAX     0xb0   // 2byte*12   (0xb0 - 0xc7)
#define  IR_ARR_MIN     0xc8   // 2byte*12   (0xc8 - 0xdf)

// Soccer Topboard Register Address
#define  ULT_DATA       0x30   // 2byte*4    (0x30 - 0x37)
#define  LED_RGB        0xa0   // 4byte*8 32 (0xa0 - 0xbf)
#define  LED_HSV        0xc0   // 4byte*8 32 (0xc0 - 0xdf)

// TFT Display Pins
#define TFT_CS  0   // TFT LCD的CS PIN腳
#define TFT_DC  53  // TFT DC(A0、RS) 
#define TFT_RST 50  // TFT Reset
#define TFT_SCL 52  // TFT SCL
#define TFT_SDA 51  // TFT SDA

// =============================================================================
//                              Type Definitions
// =============================================================================

typedef enum {
  CL1, CL2, CL3, CL4, CL5, CL6, CL7, CL8
} CL_SENSOR;

typedef enum {
  U1, U2, U3, U4
} ULTR_SENSOR;

typedef enum {
  S1_P = 10, S2_P, S3_P, S4_P
} S_PIN;

typedef enum {
  D6_P = 56, D5_P, D4_P, D3_P, D2_P, D1_P
} D_PIN;

typedef enum {
  A4_P = 62, A3_P, A2_P, A1_P
} A_PIN;

typedef enum {
  BLACK, WHITE, GREY, RED, GREEN, BLUE, YELLOW, CYAN
} color_sensor_color;

// =============================================================================
//                              Main Class
// =============================================================================

class PeanutKingSoccerV4 {
public:
  // Constructor
  PeanutKingSoccerV4(void);

  // Initialize the robot's modules
  void init(uint8_t = 0);

	// Read all sensor data from the robot's modules
  void dataFetch(void);

// =============================================================================
//                      Module Instances
// =============================================================================

  ButtonManager buttonMgr;  // ButtonManager instance for reading button states
  LedController ledCtrl;    // LedController instance for controlling on-board LEDs
  Motor     motor;    // Motor instance for controlling the robot's motors
  Movement  move;     // Movement instance for controlling the robot's movement
  Compass   compass;  // Compass instance for reading compass heading
  PDQ_ST7735 tft;     // TFT display instance for displaying graphics and text

// =============================================================================
//                Button Functions (wrapper for compatibility)
// =============================================================================

  /**
   * Read button state (pressed or not)
   * `btn` - Button ID (`BTN_1` - `BTN_4`)
   *
   * `Returns` - `true` if pressed, `false` otherwise
   */
  bool buttonRead(BUTTON_ID btn);
  /**
   * Update button state machine
   * Should be called regularly to detect `TAP`, `PRESS`, `HOLD`, etc.
   */
  void buttonUpdate(void);
  /**
   * Get the current status of a button
   * `btn` - Button ID (`BTN_1` - `BTN_4`)
   * 
   * `Returns` - Current button status, e.g., `TAP`, `PRESS`, `HOLD`, etc.
   */
  buttonStatus_t buttonGetStatus(BUTTON_ID btn);

// =============================================================================
//                    IR Compound Eye Functions
// =============================================================================

  uint8_t* compoundEyeRead();
  uint8_t  compoundMaxEye(void);
  uint8_t  compoundMaxEyeVal(void);
  uint8_t  compoundEyeVal(uint8_t n);
  void     compoundEyeCal(float* calData);

// =============================================================================
//                     Color Sensor Functions
// =============================================================================

  uint8_t  getColorSensor(uint8_t);
  rgb_t    getColorSensorRGB(uint8_t);
  hsl_t    getColorSensorHSL(uint8_t);
  uint8_t  floorColorReadRaw(uint8_t, uint8_t = 0);
  uint16_t floorColorRead(uint8_t);
  uint16_t getRedColor(uint8_t i);
  uint8_t  colorReadAll(void);
  uint16_t whiteLineCal(uint8_t = 00);
  bool     whiteLineCheck(uint8_t, uint16_t);
  void     setColorBL(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

// =============================================================================
//                      Ultrasonic Functions
// =============================================================================

  uint16_t ultrasonicRead(uint8_t);

// =============================================================================
//                LED Functions (wrapper for compatibility)
// =============================================================================

  /**
   * Set all on-board LEDs to a specific color
   * `color` - Color to set (`LED_OFF` - `LED_WHITE`)
   */
  void setOnBrdLED(uint8_t color);
  /**
   * Set a single on-board LED `on`/`off`
   * `LED`    - LED index (`0-2`)
   * `status` - `0` = off, `1` = on
   */
  void setOnBrdLED(uint8_t LED, uint8_t status);

// =============================================================================
//                     TFT Display Functions
// =============================================================================

  void setScreen(uint8_t col, uint8_t row, char string[]);
  void setScreen(uint8_t col, uint8_t row, int16_t numbers);
  void clearScreen(void);

// =============================================================================
//                      Bluetooth Functions
// =============================================================================

  void bluetoothRemote(void);
  void bluetoothAttributes(void);

// =============================================================================
//              Compass Functions (wrapper for compatibility)
// =============================================================================

  /**
   * Read the compass heading
   *
   * `Returns` - heading in degrees `(0~360°)`, clockwise
   */
  uint16_t compassRead(void);
  /**
   * Get raw accelerometer data
   *
   * `Returns` - array of `accelData[3]` (X, Y, Z)
   */
  int16_t* getAccelerometerRaw(void);
  /**
   * Get raw gyroscope data
   *
   * `Returns` - array of `gyroData[3]` (X, Y, Z)
   */
  int16_t* getGyroscopeRaw(void);
  /**
   * Get raw magnetometer data
   *
   * `Returns` - array of `magData[3]` (X, Y, Z)
   */
  int16_t* getMagnetometerRaw(void);

// =============================================================================
//                Motor Functions (wrapper for compatibility)
// =============================================================================

  /**
   * Set the speed of a single motor
   * `mi`     - Motor ID ( `M1` - `M4` )
   * `speed`  - Speed `(0~255)`, positive=`CCW,` negative=`CW`, `0`=`brake`
   */
  void setMotorSpeed(MOTOR_ID mi, int16_t speed);
  /**
   * Stop all motors (brake mode)
   */
  void stopAllMotors(void);

// =============================================================================
//               Movement Functions (wrapper for compatibility)
// =============================================================================

  /**
   * Move robot at angle with speed and rotation
   * `mAngle`  - Movement angle `(0-360°)`
   * `mSpeed`  - Movement speed `(0-255)`
   * `rotate`  - Rotation speed `(-255 to +255)`, positive=`CW`, negative=`CCW`
   */
  void moveByAngle(float mAngle, float mSpeed, float rotate);
  /**
   * Move robot with compass correction and speed scaling
   * `mAngle`          - Movement angle `(0-360°)`
   * `mSpeed`          - Movement speed `(0-255)`
   */
  void moveByAnglePID(float mAngle, float mSpeed);

// =============================================================================
//                      Strategy Functions
// =============================================================================

  void Chase(int& direct, int& speed, int& rotation);
  void Back(int& direct, int& speed, int& rotation);

// =============================================================================
//                    I2C Low-Level Functions
// =============================================================================

  IICIT::status_t rxCpltCallback(const IICIT::status_t status);
  void enableScanning(bool, uint16_t, bool);
  void I2CSensorRead(IICIT::Handle handle, uint8_t sensor, uint8_t length);
  void I2CSensorSend(IICIT::Handle handle, uint8_t sensor, uint8_t *data, uint8_t length);

// =============================================================================
//                      Public Sensor Data
// =============================================================================

  // Compass
  uint16_t heading;   // Compass heading (0~360 degrees)

  // Compound eye
  uint8_t  eye[12];   // 12 IR readings
  uint16_t eyeAngle;  
  uint8_t  maxEye;    // Index of the maximum IR reading

  // Color sensor
  uint8_t  groundColor[4];
  rgb_t    colorRGB[8];
  hsl_t    colorHSL[8];
  hsv_t    colorHSV[8];
  bool     isWhite[8] = {false};
  uint16_t whiteLineThreshold[8] = {30, 30, 30, 30};

  // Ultrasonic
  uint16_t ultrasonic[4];

  // Bluetooth data
  uint8_t  btButton[10];
  uint8_t  btButtonIndex;
  uint8_t  btGestureCode;
  uint8_t  btButtonFunction[4];
  uint8_t  btAttributes[5] = {5, 5, 5, 5, 5};
  uint8_t  btTxBuffer[50];
  uint8_t  btRxBuffer[50];
  int16_t  btDegree = 0;
  int16_t  btDistance = 0;
  int16_t  btRotate = 0;

  // Misc
  uint16_t EYEBOUNDARY = 20;
  uint16_t systemTime;
  uint32_t screenTicks = 0;
  uint32_t sysTicks = 0;
  uint16_t tim1Count = 0;

// =============================================================================
//                        Constants
// =============================================================================

  const int8_t  PAGEUPPERLIMIT = 6;
  const int8_t  PAGELOWERLIMIT = 0;
  const uint8_t sensorBoardAddr = 0x13;

private:
// =============================================================================
//                        I2C Handles
// =============================================================================

  IICIT::Handle senbrdHandle;   // I2C handle for the sensor board
  IICIT::Handle topbrdHandle;   // I2C handle for the top board

// =============================================================================
//                        I2C Buffers
// =============================================================================

  uint8_t rxBuff[50];   // Buffer for I2C read operations
  uint8_t txBuff[50];   // Buffer for I2C write operations

// =============================================================================
//                        Pin Allocation
// =============================================================================

  const uint8_t APin[4];
  const uint8_t DPin[6];
  const uint8_t pwmPin[4];
  const uint8_t ULTPin_trig[4];
  uint8_t ULTPin_echo[4];

// =============================================================================
//                        Software I2C Instances
// =============================================================================

  SlowSoftI2CMaster swiic[8];   // Software I2C instances for color sensors

// =============================================================================
//                        Internal Ultrasonic ISR
// =============================================================================

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

#endif // PeanutKing_Soccer_V4_H