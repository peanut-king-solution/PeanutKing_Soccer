#ifndef COMPOUND_EYE_H
#define COMPOUND_EYE_H

#include <Arduino.h>
#include "modules/I2C/i2cManager.h"

// IR Compound Eye Register Addresses
#define IR_RAW     0x00    // 12 bytes (raw readings)
#define IR_MAX     0x11    // max value index
#define IR_MIN     0x12    // min value index
#define IR_ANGLE   0x13    // 2 bytes
#define IR_COUNT   0x2d    // 2 bytes
#define IR_LEDEN   0x2f
#define IR_CAL     0x30    // 24 bytes (calibration data)
#define IR_ARR_MAX 0xb0    // 24 bytes
#define IR_ARR_MIN 0xc8    // 24 bytes

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
   * Calibrate the IR sensors with calibration data
   * Writes calibration values to sensor board registers
   * `calData` - Array of 12 calibration float values
   */
  void calibrate(float *calData);
};

#endif // COMPOUND_EYE_H
