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

// Include all module headers
#include "modules/Motor/Motor.h"
#include "modules/Movement/Movement.h"
#include "modules/ColorSensor/ColorSensor.h"
#include "modules/CompoundEye/CompoundEye.h"
#include "modules/ButtonManager/ButtonManager.h"
#include "modules/Led/LED.h"
#include "modules/Ultrasonic/Ultrasonic.h"
#include "modules/Compass/Compass.h"
#include "modules/Bluetooth/Bluetooth.h"
#include "modules/PS2/PS2.h"

// Include external libraries
#include <SPI.h>                      // SPI library for TFT and PS2
#include "modules/TFT/PDQ_GFX.h"      // PDQ: Core graphics library
#include "modules/TFT/PDQ_ST7735.h"   // PDQ: Hardware-specific driver library

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
#define DEBUGMODE 1

// =============================================================================
//                              Type Definitions
// =============================================================================

typedef enum {
  S1_P = 10, S2_P, S3_P, S4_P
} S_PIN;

typedef enum {
  // D6_P = 56, D5_P, D4_P, D3_P, D2_P, D1_P
  D6_P = A2, D5_P, D4_P, D3_P, D2_P, D1_P = A7
} D_PIN;

typedef enum {
  // A4_P = 62, A3_P, A2_P, A1_P
  A4_P = A8, A3_P, A2_P, A1_P = A11
} A_PIN;

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

  Motor         motor;        // Motor instance for controlling the robot's motors
  Movement      move;         // Movement instance for controlling the robot's movement
  ColorSensor   colorSensor;  // ColorSensor instance for reading color sensors
  CompoundEye   compoundEye;  // CompoundEye instance for reading IR sensors
  ButtonManager buttonMgr;    // ButtonManager instance for reading button states
  LED           led;          // LED instance for controlling on-board LEDs
  Ultrasonic    xsound;       // Ultrasonic instance for managing 4 ultrasonic sensors
  Compass       compass;      // Compass instance for reading compass heading
  Bluetooth     bluetooth;    // Bluetooth instance for managing Bluetooth Low Energy
  PDQ_ST7735    tft;          // TFT display instance for displaying graphics and text
  PS2X          ps2x;         // PS2X instance for reading PS2 controller inputs

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
   * Move robot with compass correction
   * `mAngle`          - Movement angle `(0-360°)`
   * `mSpeed`          - Movement speed `(0-255)`
   */
  void moveWithCorr(float mAngle, float mSpeed);

// =============================================================================
//                     Color Sensor Functions (wrapper)
// =============================================================================

  /**
   * Read color index from a sensor
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - Color index (`0`=Black ... `7`=Cyan)
   */
  uint8_t  getColorSensor(CLR_SENSOR_ID sensorNum);
  /**
   * Read RGB values from a sensor
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - RGB structure
   */
  rgb_t    getColorSensorRGB(CLR_SENSOR_ID sensorNum);
  /**
   * Read HSL values from a sensor
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - HSL structure
   */
  hsl_t    getColorSensorHSL(CLR_SENSOR_ID sensorNum);
  /**
   * Check if a sensor detects white line
   * `i`      - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - `true` if white line detected
   */
  bool     whiteLineCheck(CLR_SENSOR_ID i);

// =============================================================================
//                    IR Compound Eye Functions (wrapper)
// =============================================================================

  /**
   * Read all 12 IR sensor values
   *
   * `Returns` - Pointer to the `eye[12]` array
   */
  uint8_t* compoundEyeRead();
  /**
   * Get the index of the IR sensor with maximum reading
   *
   * `Returns` - Index `(0-11)` of the sensor with max value
   */
  uint8_t  compoundMaxEye(void);
  /**
   * Get the maximum IR sensor value
   *
   * `Returns` - Maximum value among all 12 sensors
   */
  uint8_t  compoundMaxEyeVal(void);
  /**
   * Get the value of a specific IR sensor
   * `n` - Sensor index `(0-11)`
   *
   * `Returns` - IR sensor value
   */
  uint8_t  compoundEyeVal(uint8_t n);
  /**
   * Get the angle of the detected object
   *
   * `Returns` - Angle in degrees `(0-360)`
   */
  uint16_t compoundEyeAngle(void);

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
//                LED Functions (wrapper for compatibility)
// =============================================================================

  /**
   * Set all on-board LEDs to a specific color
   * `color` - Color to set (`LEDColor::OFF` - `LEDColor::WHITE`)
   */
  void setOnBrdLED(LEDColor color);
  /**
   * Set a single on-board LED `on`/`off`
   * `LED`    - LED index (`0-2`)
   * `status` - `0` = off, `1` = on
   */
  void setOnBrdLED(uint8_t LED, uint8_t status);

// =============================================================================
//                      Ultrasonic Functions (wrapper for compatibility)
// =============================================================================

  /**
   * Read the distance from an ultrasonic sensor
   * `n` - Sensor ID (`U1` - `U4`)
   *
   * `Returns` - Distance in mm (0~4500mm)
   */
  uint16_t ultrasonicRead(ULTR_SENSOR n);

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
//                     TFT Display Functions
// =============================================================================

  /**
   * Set the foreground color for text on the TFT display
   * `color` - 16-bit color value (e.g., `ST7735_WHITE`, `ST7735_YELLOW`, `ST7735_BLACK`)
   */
  void setTextColor(uint16_t color);
  /**
   * Set the foreground and background color for text on the TFT display
   * `fg` - Foreground color
   * `bg` - Background color
   */
  void setTextColor(uint16_t fg, uint16_t bg);
  /**
   * Set the text size for the TFT display
   * `size` - Text size
   */
  void setTextSize(uint8_t size);

  /**
   * Display a text on the TFT screen at the specified column and row
   * `col` - Column position (`0-based`, each column is `6 pixels` wide)
   * `row` - Row position (`0-based`, each row is `10 pixels` high)
   * `string` - Text string to display
   */
  void setScreen(uint8_t col, uint8_t row, char string[]);
  /**
   * Display a `number` on the TFT screen at the specified column and row
   * `col` - Column position (`0-based`, each column is `6 pixels` wide)
   * `row` - Row position (`0-based`, each row is `10 pixels` high)
   * `numbers` - Number to display
   */
  void setScreen(uint8_t col, uint8_t row, int16_t numbers);
  /**
   * Clear the entire TFT screen (fill with `black`)
   */
  void clearScreen(void);
  /**
   * Draw a angle pointer (arrow + circle + N/S/E/W markers) on the TFT screen
   * `x` - Center X coordinate of the compass (pixels)
   * `y` - Center Y coordinate of the compass (pixels)
   * `radius` - Radius of the compass circle (pixels)
   * `angle` - Angle in degrees (`0~360`), clockwise
   * `arrowColor` - Color of the pointer arrow (default: `ST7735_MAGENTA`)
   */
  void drawAnglePointer(int x, int y, int radius, uint16_t angle, uint16_t arrowColor = ST7735_MAGENTA);

// =============================================================================
//                      Bluetooth Functions
// =============================================================================

private:
  bool _sendPILAData(void);
  void _PILAUpdate(void);
  void _DASHBOARDUpdate(void);
public:
  /**
   * Handle Bluetooth remote control commands (PILA mode / Dashboard mode)
   * 
   */
  void bluetoothRemote(void);

// =============================================================================
//                     PS2 Controller Functions
// =============================================================================

  /**
   * Initialize the PS2 controller, there are 4 pins need to be connected to the PS2 controller
   * but you only need to specify the `CLK` and `DAT` pins which are the start pin and end pin
   * Because the other 2 pins will be automatically assigned.
   * `CLK`      - Clock pin (digital pin: D0_P ~ D5_P)
   * `DAT`      - Data pin  (digital pin: D0_P ~ D5_P)
   * `pressure` - Enable pressure-sensitive buttons (default: `false`)
   * `vibration`- Enable vibration feedback (default: `false`)
   * 
   * `Returns` - `0` if successful, error code otherwise
   */
  byte ps2Init(D_PIN CLK = D1_P, D_PIN DAT = D4_P,bool pressure = false, bool vibration = false);
  /**
   * Set the vibration strength of the PS2 controller
   * `strength` - Vibration strength (0-255)
   */
  void ps2SetVibration(byte strength);
  /**
   * Update the PS2 controller state
   */
  void ps2Update(void);
  /**
   * Read the state of a PS2 controller button
   * `button` - Button ID (e.g., `PS2Button::CROSS`, `PS2Button::CIRCLE`, etc.)
   *
   * `Returns` - `true` if pressed, `false` otherwise
   */
  PS2ButtonState ps2ButtonRead(PS2Button button);
  /**
   * Check if a PS2 controller button was just pressed
   * `button` - Button ID (e.g., `PS2Button::CROSS`, `PS2Button::CIRCLE`, etc.)
   *
   * `Returns` - `true` if just pressed, `false` otherwise
   */
  bool ps2ButtonPressed(PS2Button button);
  /**
   * Check if a PS2 controller button is being held down
   * `button` - Button ID (e.g., `PS2Button::CROSS`, `PS2Button::CIRCLE`, etc.)
   *
   * `Returns` - `true` if being held, `false` otherwise
   */
  bool ps2ButtonHolding(PS2Button button);
  /**
   * Check if a PS2 controller button was just released
   * `button` - Button ID (e.g., `PS2Button::CROSS`, `PS2Button::CIRCLE`, etc.)
   *
   * `Returns` - `true` if just released, `false` otherwise
   */
  bool ps2ButtonReleased(PS2Button button);
  /**
   * Read the data of a PS2 controller joystick
   * `joystick` - Joystick ID (e.g., `PS2Joystick::LX`, `PS2Joystick::LY`, etc.)
   *
   * `Returns` - Structure containing angle and strength of the joystick
   */
  PS2JoystickData ps2JoystickRead(PS2Joystick joystick);

// =============================================================================
//                      Strategy Functions
// =============================================================================

  void Chase(int& direct, int& speed, int& rotation);
  void Back(int& direct, int& speed, int& rotation);

// =============================================================================
//                      Public Sensor Data
// =============================================================================

  // Color sensor
  rgb_t    colorRGB[8];   // RGB values
  hsl_t    colorHSL[8];   // HSL values
  bool     isWhite[8] = {0};  // Set all to false
  uint16_t whiteLineThreshold[8] = {30, 30, 30, 30, 30, 30, 30, 30};

  // Compound eye
  uint8_t  eye[12];    // 12 IR readings
  uint16_t eyeAngle;   // Ball angle (0~360 degrees)
  uint8_t  maxEye;     // Index of the maximum IR reading
  uint8_t  maxEyeVal;  // Maximum IR reading value
  
  // Ultrasonic
  uint16_t ultrasonic[4]; // Ultrasonic readings (U1~U4)

  // Compass
  uint16_t heading;   // Compass heading (0~360 degrees)

  // Bluetooth data
private:
  uint32_t _lastSendTime = 0;  // Timestamp of the last data sent via Bluetooth

  // PS2 controller data
  byte vibrationStr = 0;  // Vibration strength (0-255)

private:
// =============================================================================
//                        Pin Allocation
// =============================================================================

  const uint8_t pwmPin[4];
};

#endif // PeanutKing_Soccer_V4_H