#ifndef COMPOUND_EYE_H
#define COMPOUND_EYE_H

#include <Arduino.h>
#include "modules/I2C/i2cManager.h"

// IR Compound Eye Register Addresses
// Command Map (in decimal):
//  0-11 : Reading of IR1-IR12 (0-255)
//  12   : Maximum IR reading  (0-255)
//  13   : Maximum IR          (1-12)
//  14   : Angle               (0-360)
//  15   : Mode                (0=single, 1=double)
#define IR_RAW     0x00    // 12 bytes (IR1-IR12 readings)
#define IR_MAX     0x0C    // 1 byte  (maximum IR value)
#define IR_MAX_IDX 0x0D    // 1 byte  (maximum IR index)
#define IR_ANGLE   0x0E    // 2 bytes (angle 0-360)
#define IR_MODE    0x0F    // 1 byte  (0=single, 1=double)

// Default I2C address for the sensor board
#define COMPOUND_EYE_I2C_ADDRESS 0x13
/**
 * CompoundEye class for reading IR sensor values via Hardware I2C
 * Manages 12 IR sensors arranged in a circle for ball detection
 */
class CompoundEye
{
private:
  uint8_t    _address;  // I2C address
  I2C_Handle _handle;   // I2C handle
  uint8_t    _eye[12];  // Internal buffer for 12 IR readings

public:
  /**
   * Constructor
   * `address` - I2C address of the sensor board (default: `0x13`)
   */
  CompoundEye(uint8_t address = COMPOUND_EYE_I2C_ADDRESS);

  /**
   * Initialize the compound eye module
   *
   * `Returns` - `true` if successful, `false` otherwise
   */
  bool init(void);

  /**
   * Read all 12 IR sensor values
   *
   * `Returns` - Pointer to the internal `_eye[12]` array
   */
  uint8_t* readAll(void);

  /**
   * Get the index of the IR sensor with maximum reading
   *
   * `Returns` - Index `(0-11)` of the sensor with max value
   */
  uint8_t getMaxEye(void);

  /**
   * Get the maximum IR sensor value
   *
   * `Returns` - Maximum value among all 12 sensors
   */
  uint8_t getMaxEyeVal(void);

  /**
   * Get the value of a specific IR sensor
   * `n` - Sensor index `(0-11)`
   *
   * `Returns` - IR sensor value
   */
  uint8_t getEyeVal(uint8_t n);

  /**
   * Get the angle of the detected object
   *
   * `Returns` - Angle in degrees `(0-360)`
   */
  uint16_t getAngle(void);

  /**
   * Get the detection mode
   *
   * `Returns` - Mode `(0=single, 1=double)`
   */
  uint8_t getMode(void);
};

#endif // COMPOUND_EYE_H
