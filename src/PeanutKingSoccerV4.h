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
#include "modules/Button/Button.h"
#include "modules/Led/LED.h"
#include "modules/Ultrasound/Ultrasound.h"
#include "modules/Compass/Compass.h"
#include "modules/Bluetooth/Bluetooth.h"
#include "modules/PS2/PS2.h"

#include "modules/I2C/i2cManager.h"

// Include utility libraries
#include "utils/Converter.h"
#include "utils/PIDController.h"

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

enum S_PIN {
  S1_P = 10, S2_P, S3_P, S4_P
};

enum DigitalPinId {
  D6_P = A2, D5_P, D4_P, D3_P, D2_P, D1_P = A7
};

enum AnalogPinId {
  A4_P = A8, A3_P, A2_P, A1_P = A11
};

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
  Movement      movement;     // Movement instance for controlling the robot's movement
  ColorSensor   colorSensor;  // ColorSensor instance for reading color sensors
  CompoundEye   compoundEye;  // CompoundEye instance for reading IR sensors
  Button        button;       // Button instance for reading button states
  LED           led;          // LED instance for controlling on-board LEDs
  Ultrasound    ultrasound;   // Ultrasound instance for managing 4 ultrasonic sensors
  Compass       compass;      // Compass instance for reading compass heading
  Bluetooth     bluetooth;    // Bluetooth instance for managing Bluetooth Low Energy
  PDQ_ST7735    tft;          // TFT display instance for displaying graphics and text
  PS2X          ps2x;         // PS2X instance for reading PS2 controller inputs

// =============================================================================
//                    Coordinate System (generic, template)
// =============================================================================

  /**
   * Reset the coordinate system of any module with a `converter` member.
   * `module` - reference to the module (e.g. `robot.movement`, `robot.compass`, `robot.compoundEye`)
   */
  template<typename T>
  void coordinateReset(T& module) { module.converter.reset(); }

  /**
   * Rotate the coordinate system of any module with a `converter` member.
   * `module` - reference to the module
   * `angle`  - non-negative rotation angle in degrees [0, 360)
   * `dir`    - rotation direction (`CW` by default)
   */
  template<typename T>
  void coordinateRotate(T& module, uint16_t angle, RotationDir dir = CW) { module.converter.rotate(angle, dir); }

  /**
   * Flip the coordinate system direction (CW <-> CCW) of any module with a `converter` member.
   * `module` - reference to the module
   */
  template<typename T>
  void coordinateFlip(T& module) { module.converter.flip(); }

// =============================================================================
//                Motor Functions (Driven by Motor position)
// =============================================================================
  
  /**
   * Configure which motor port ( `M1` - `M4` ) controls which wheel position
   * `RightFront` - Motor port for right front wheel (default: `M1`)
   * `RightBack`  - Motor port for right back wheel (default: `M2`)
   * `LeftBack`   - Motor port for left back wheel (default: `M3`)
   * `LeftFront`  - Motor port for left front wheel (default: `M4`)
   */
  void motorConfiguration(MotorId RightFront, MotorId RightBack, MotorId LeftBack, MotorId LeftFront);

  /**
   * Flip the rotation direction of a single motor
   * `pos`    - Motor position ( `RightFront` - `LeftFront` )
   * `flip`   - `true`=flip, `false`=normal (default: `true`)
   */
  void motorFlipDirection(MotorPos pos, bool flip = true);

  /**
   * Set the speed of a single motor
   * `pos`    - Motor position ( `RightFront` - `LeftFront` )
   * `speed`  - Speed `(0~255)`, positive=`CCW,` negative=`CW`, `0`=`brake`
   */
  void motorSetSpeed(MotorPos pos, int16_t speed);

  /**
   * Stop a single motor
   * `pos`  - Motor position ( `RightFront` - `LeftFront` )
   */
  void motorStop(MotorPos pos);

  /**
   * Stop all motors (brake mode)
   */
  void motorStopAll(void);

  /**
   * Test all motors sequentially ( `RightFront` -> `RightBack` -> `LeftBack` -> `LeftFront` )
   * `speed` - Test speed `(0~255)`
   * `duration` - Duration for each motor test in milliseconds
   */
  void motorTestAll(int16_t speed, int duration = 1000);

// =============================================================================
//                           Movement Functions
// =============================================================================

  bool compassCorrectEnabled = true;   // Enable/disable compass correction for movement
  bool outBoundPreventEnabled = false;  // Enable/disable out-of-bounds prevention for movement

  /**
   * Move the robot in a specified direction with optional rotation
   * `mAngle` - Movement angle (0-360 degrees)
   * `mSpeed` - Movement speed (0-255)
   * `rotate` - Rotation speed (-255 to +255)
   *
   * `Note:` The method automatically selects the appropriate movement algorithm based on the enabled features
   * 1. `compassCorrectEnabled`, `outBoundPreventEnabled` = true → compass correction + out-of-bounds prevention
   * 2. `compassCorrectEnabled` = true only → compass correction only
   * 3. `outBoundPreventEnabled` = true only → out-of-bounds prevention only
   * 4. both = false → direct movement without corrections
   *
   * `Note:` Out-of-bounds prevention is not yet implemented, so `outBoundPreventEnabled`
   * defaults to `false`. Enable it only when the algorithm is ready.
   */
  void move(float mAngle, float mSpeed, float rotate = 0);

  /**
   * Test the movement of the robot ( `forward` -> `right front` -> `rightward` )
   * `speed` - Test speed `(0-255)`
   */
  void moveTest(float speed);

// =============================================================================
//                     Color Sensor Functions (wrapper)
// =============================================================================

  /**
   * Assign which color sensor port (`CL1`-`CL8`) corresponds to which physical position
   * `Front` - Color sensor port at front position
   * `Right` - Color sensor port at right position
   * `Back`  - Color sensor port at back position
   * `Left`  - Color sensor port at left position
   */
  void colorSensorConfiguration(ColorSensorId Front, ColorSensorId Right, ColorSensorId Back, ColorSensorId Left);
  /**
   * Read raw RGBC values from a color sensor at the specified position
   * `pos` - Physical position (`Front`, `Right`, `Back`, `Left`)
   *
   * `Returns` - RGBC raw structure (0-65535 each)
   */
  RGBC colorSensorReadRGBC(SensorPos pos);
  /**
   * Read RGB values from a color sensor at the specified position
   * `pos` - Physical position (`Front`, `Right`, `Back`, `Left`)
   *
   * `Returns` - RGB structure (0-255 each)
   */
  RGB colorSensorReadRGB(SensorPos pos);
  /**
   * Read HSL values from a color sensor at the specified position
   * `pos` - Physical position (`Front`, `Right`, `Back`, `Left`)
   *
   * `Returns` - HSL structure
   */
  HSL colorSensorReadHSL(SensorPos pos);
  /**
   * Check if a color sensor at the specified position detects a white line
   * `pos` - Physical position (`Front`, `Right`, `Back`, `Left`)
   *
   * `Returns` - `true` if white line detected, `false` otherwise
   */
  bool isWhiteLine(SensorPos pos);
  /**
   * Calibrate the baseline for a color sensor at the specified position
   * `pos` - Physical position (`Front`, `Right`, `Back`, `Left`)
   * `samples` - Number of samples to take for calibration (default: 10)
   */
  void colorSensorCalBaseline(SensorPos pos, uint8_t samples = 10);
  /**
   * Get the calibrated baseline for a color sensor
   * `pos` - Physical position (`Front`, `Right`, `Back`, `Left`)
   *
   * `Returns` - `GreenBaseline` struct (check `.calibrated` before use)
   */
  GreenBaseline colorSensorGetBaseline(SensorPos pos);

// =============================================================================
//                    IR Compound Eye Functions (wrapper)
// =============================================================================

  /**
   * Read all 12 IR sensor values
   *
   * `Returns` - Pointer to the `eyes[12]` array
   */
  uint8_t* compoundEyeReadAll(void);
  /**
   * Get the index of the eye with maximum reading
   *
   * `Returns` - `EyeId` of the eye with max value
   */
  EyeId compoundMaxEyeRead(void);
  /**
   * Get the maximum IR sensor value
   *
   * `Returns` - Maximum value among all 12 sensors
   */
  uint8_t compoundMaxEyeValueRead(void);
  /**
   * Get the value of a specific eye
   * `eyeIndex` - Eye index ( `Eye0` - `Eye11` )
   *
   * `Returns` - IR sensor value
   */
  uint8_t compoundEyeValueRead(EyeId eyeIndex);
  /**
   * Get the angle of the detected object
   *
   * `Returns` - Angle in degrees `(0-360)`
   */
  uint16_t compoundEyeAngleRead(void);

  uint8_t compoundEyeModeRead(void);

// =============================================================================
//                Button Functions (wrapper for compatibility)
// =============================================================================

  /**
   * Update the button state machine
   * This should be called regularly to update the button states.
   */
  void buttonUpdate(void);

  /**
   * Get the current state of a button, should be called after buttonUpdate() to get the latest state.
   * `btn` - Button ID (`Button1` - `Button4`)
   *
   * `Returns` - Current button state, e.g., `ButtonIdle`, `ButtonPressed`, `ButtonHolding`, `ButtonReleased`
   */
  ButtonState buttonStateRead(ButtonId btn);

// =============================================================================
//                LED Functions (wrapper for compatibility)
// =============================================================================

  /**
   * Set all on-board LEDs to a specific color
   * `color` - Color to set (`LEDOff` - `LEDWhite`)
   */
  void onBoardLedSet(LEDColor color);
  /**
   * Set a single channel on-board LED `on`/`off`
   * `LED`    - RGB Pin index (`0`=Blue, `1`=Green, `2`=Red)
   * `status` - `LOW` = off, `HIGH` = on
   */
  void onBoardLedSet(uint8_t LED, uint8_t status);

// =============================================================================
//                      Ultrasound Functions (wrapper for compatibility)
// =============================================================================

  /**
   * Get distance from a specified position
   *
   * `pos` - Position (Front, Right, Back, Left)
   *
   * `Returns` - Distance in mm (0~4500mm)
   */
  uint16_t ultrasoundGetDist(SensorPos pos);

  /**
   * Configure which ultrasound port (U1~U4) is at each physical position
   *
   * `front` - ultrasound port plugged at the front position
   * `right` - ultrasound port plugged at the right position
   * `back`  - ultrasound port plugged at the back position
   * `left`  - ultrasound port plugged at the left position
   */
  void ultrasoundConfiguration(UltrasoundId Front, UltrasoundId Right, UltrasoundId Back, UltrasoundId Left);

  /**
   * Enable or disable sensors by position
   *
   * `front` - `true` to enable front sensor, `false` to disable
   * `right` - `true` to enable right sensor, `false` to disable
   * `back`  - `true` to enable back sensor, `false` to disable
   * `left`  - `true` to enable left sensor, `false` to disable
   */
  void ultrasoundSetEnabled(bool front, bool right, bool back, bool left);
  /**
   * Enable or disable all ultrasound sensors
   *
   * `enabled` - `true` to enable all sensors, `false` to disable all
   */
  void ultrasoundEnableAll(bool enabled);

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
   * Read raw accelerometer data
   *
   * `Returns` - array of `accelData[3]` (X, Y, Z)
   */
  int16_t* compassReadRawAccel(void);
  /**
   * Read raw gyroscope data
   *
   * `Returns` - array of `gyroData[3]` (X, Y, Z)
   */
  int16_t* compassReadRawGyro(void);
  /**
   * Read raw magnetometer data
   *
   * `Returns` - array of `magData[3]` (X, Y, Z)
   */
  int16_t* compassReadRawMag(void);

  /**
   * Sets the current heading as the new north (0°) reference.
   * It adjusts the factory converter offset to account for the new north offset.
   */
  void compassUpdateNorth(void);

// =============================================================================
//                     TFT Display Functions
// =============================================================================

  /**
   * Set the foreground color for text on the TFT display
   * `color` - 16-bit color value (e.g., `ST7735_WHITE`, `ST7735_YELLOW`, `ST7735_BLACK`)
   */
  void screenSetTextColor(uint16_t color);
  /**
   * Set the foreground and background color for text on the TFT display
   * `fg` - Foreground color
   * `bg` - Background color
   */
  void screenSetTextColor(uint16_t fg, uint16_t bg);
  /**
   * Set the text size for the TFT display
   * `size` - Text size
   */
  void screenSetTextSize(uint8_t size);
  /**
   * Display a text on the TFT screen at the specified column and row
   * `col` - Column position (`0-based`, each column is `6 pixels` wide)
   * `row` - Row position (`0-based`, each row is `10 pixels` high)
   * `string` - Text string to display
   */
  void screenPrintText(uint8_t col, uint8_t row, const char* string);
  /**
   * Display a `number` on the TFT screen at the specified column and row
   * `col` - Column position (`0-based`, each column is `6 pixels` wide)
   * `row` - Row position (`0-based`, each row is `10 pixels` high)
   * `numbers` - Number to display
   */
  void screenPrintNumber(uint8_t col, uint8_t row, int16_t number);
  /**
   * Clear the entire TFT screen (fill with `black`)
   */
  void screenClear(void);
  /**
   * Draw a angle pointer (arrow + circle + N/S/E/W markers) on the TFT screen
   * `x` - Center X coordinate of the compass (pixels)
   * `y` - Center Y coordinate of the compass (pixels)
   * `radius` - Radius of the compass circle (pixels)
   * `angle` - Angle in degrees (`0~360`), clockwise
   * `arrowColor` - Color of the pointer arrow (default: `ST7735_MAGENTA`)
   * 
   * `Note:` This method will change the text size and color settings of the TFT display, 
   * so you may need to reset them after calling this method if you want to continue using text functions.
   */
  void screenDrawAnglePointer(int x, int y, int radius, uint16_t angle, uint16_t arrowColor = ST7735_MAGENTA);

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
  byte ps2Init(DigitalPinId CLK = D1_P, DigitalPinId DAT = D4_P,bool pressure = false, bool vibration = false);
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
   * Get the current state of a PS2 button
   * `button` - Button ID (e.g., `PS2Cross`, `PS2Circle`, etc.)
   *
   * `Returns` - Current button state, e.g., `PS2Idle`, `PS2Pressed`, `PS2Holding`, `PS2Released`
   */
  PS2ButtonState ps2ButtonStateRead(PS2Button button);
  /**
   * Read the data of a PS2 controller joystick
   * `joystick` - Joystick ID (e.g., `PS2LeftJoystick`, `PS2RightJoystick`)
   *
   * `Returns` - Structure containing angle and strength of the joystick
   */
  PS2JoystickData ps2JoystickRead(PS2Joystick joystick);

// =============================================================================
//                      Public Sensor Data
// =============================================================================

  // Color sensor (indices 0-3 = Front, Right, Back, Left)
  RGBC  colorRGBC[4] = {};  // RGBC raw values
  RGB   colorRGB[4]  = {};   // RGB values
  HSL   colorHSL[4]  = {};   // HSL values
  bool  isWhite[4]   = {};   // White line detection flags

  // Compound eye
  uint8_t  eyes[12]  = {};   // 12 IR readings
  uint16_t irAngle   = 0;    // Ball angle (0~360 degrees)
  uint8_t  maxEye    = 0;    // Index of the maximum IR reading
  uint8_t  maxEyeVal = 0;   // Maximum IR reading value
  
  // Ultrasound
  uint16_t distances[4] = {}; // Ultrasound readings (Front, Right, Back, Left) in mm (0~4500mm)

  // Compass
  uint16_t heading = 0; // Compass heading (0~360 degrees)

  // Bluetooth data
private:
  uint32_t _lastSendTime = 0; // Timestamp of the last data sent via Bluetooth

  // PS2 controller data
  byte vibrationStr = 0;  // Vibration strength (0-255)

private:
// =============================================================================
//                        Pin Allocation
// =============================================================================

  const uint8_t pwmPin[4];
};

#endif // PeanutKing_Soccer_V4_H