#ifndef COMPOUND_EYE_H
#define COMPOUND_EYE_H

#include <Arduino.h>
#include "modules/I2C/i2cManager.h"

// IR Compound Eye Register Addresses
// Command Map (in decimal):
//  0-11 : Reading of Eye0-Eye11 (0-255)
//  12   : Maximum IR reading  (0-255)
//  13   : Maximum IR          (1-12)
//  14   : Angle               (0-360)
//  15   : Mode                (0=single, 1=double)
#define IR_RAW     0x00    // 12 bytes (Eye0-Eye11 readings)
#define IR_MAX     0x0C    // 1 byte  (maximum IR value)
#define IR_MAX_IDX 0x0D    // 1 byte  (maximum IR index)
#define IR_ANGLE   0x0E    // 1 bytes (need *2 for angle 0-360)
#define IR_MODE    0x0F    // 1 byte  (0=single, 1=double)

// Default I2C address for the sensor board
#define COMPOUND_EYE_I2C_ADDRESS 0x13

/**
 * Compound eye eye indices (Eye0-Eye11)
 */
enum EyeId : uint8_t {
  Eye0 = 0, Eye1, Eye2, Eye3, Eye4, Eye5,
  Eye6, Eye7, Eye8, Eye9, Eye10, Eye11 = 11,
};

/**
 * CompoundEye class for reading IR sensor values via Hardware I2C
 * Manages 12 IR sensors arranged in a circle for ball detection
 */
class CompoundEye
{
  friend class PeanutKingSoccerV4;   // Only PeanutKingSoccerV4 may construct this module

private:
  I2C_Handle _handle;   // I2C handle
  uint8_t    _eye[12] = {};  // Internal buffer for 12 IR readings

  /**
   * Constructor (accessible only to the friend PeanutKingSoccerV4)
   * Uses the fixed sensor board I2C address (`COMPOUND_EYE_I2C_ADDRESS`).
   */
  CompoundEye();

public:
  /**
   * Initialize the compound eye module
   *
   * `Returns` - `true` if successful, `false` otherwise
   */
  bool init(void);

  /**
   * Check if the given eye index is valid
   * `eyeIndex` - The eye index to check
   *
   * `Returns` - `true` if the index is valid, `false` otherwise
   */
  bool indexValidCheck(EyeId eyeIndex) const;

  /**
   * Read all 12 IR sensor values
   *
   * `Returns` - Pointer to the internal `_eye[12]` array
   */
  uint8_t* readAll(void);

  /**
   * Read the index of the IR sensor with maximum reading
   *
   * `Returns` - Index `(0-11)` of the sensor with max value
   */
  EyeId readMaxEye(void);

  /**
   * Read the maximum IR sensor value
   *
   * `Returns` - Maximum value among all 12 sensors
   */
  uint8_t readMaxEyeVal(void);

  /**
   * Read the value of a specific IR sensor
   * `eyeIndex` - Sensor index `(0-11)`
   *
   * `Returns` - IR sensor value
   */
  uint8_t readEyeVal(EyeId eyeIndex);

  /**
   * Read the angle of the detected object
   *
   * `Returns` - Angle in degrees `(0-360)`
   */
  uint16_t readAngle(void);

  /**
   * Read the detection mode
   *
   * `Returns` - Mode `(0=single, 1=double)`
   */
  uint8_t readMode(void);
};

#endif // COMPOUND_EYE_H
