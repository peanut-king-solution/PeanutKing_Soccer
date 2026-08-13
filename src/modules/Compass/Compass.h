#ifndef COMPASS_H
#define COMPASS_H

#include <Arduino.h>

#include "utils/Converter.h"
#include "modules/I2C/i2cManager.h"

// Compass Module Register Address
#define  ACC_RAW        0x40    // 6byte  (0-5)
#define  GYR_RAW        0x46    // 6byte  (6-b)
#define  MAG_RAW        0x4c    // 6byte  (c-2)

#define  GET_COMPASS    0x55    // 3byte
#define  GET_ROLL       0x54    // 2byte
#define  GET_YAW        0x56    // 2byte
#define  GET_PITCH      0x58    // 2byte
#define  MAG_CENT       0x5a    // xxyyzz

// I2C address for the compass module
#define COMPASS_I2C_ADDRESS 0x08

// Size of the receive buffer for I2C read operations
#define COMPASS_RX_BUFFER_SIZE 8
// Size of the transmit buffer for I2C write operations
#define COMPASS_TX_BUFFER_SIZE 8

class Compass
{
  friend class PeanutKingSoccerV4;   // Only PeanutKingSoccerV4 may construct this module

private:
  I2C_Handle _handle; // I2C handle for communication with the compass module

  uint16_t compass = 0; // Variable to store the compass reading
  uint8_t rxBuff[COMPASS_RX_BUFFER_SIZE];  // Buffer for I2C read operations
  uint8_t txBuff[COMPASS_TX_BUFFER_SIZE];  // Buffer for I2C write operations

  Converter _factoryConverter; // Factory offset (init / 655-reset only). User rotate/flip lives in `converter` and is untouched.

  // Raw sensor data arrays
  int16_t accelData[3] = {0};  // Accelerometer raw data (X, Y, Z)
  int16_t gyroData[3]  = {0};  // Gyroscope raw data (X, Y, Z)
  int16_t magData[3]   = {0};  // Magnetometer raw data (X, Y, Z)

  /**
   * Clear the receive buffer
   */
  void clearBuffer(void);

  /**
   * Read raw 6-byte sensor data
   * `reg`      - Register address to read from
   * `dataArr`  - Output array to store data (int16_t[3])
   *
   * `Returns` - pointer to the data array
   */
  int16_t* readRaw6(uint8_t reg, int16_t* dataArr);

  /**
   * Constructor (accessible only to the friend PeanutKingSoccerV4)
   * Uses the fixed compass I2C address (`COMPASS_I2C_ADDRESS`).
   */
  Compass();

public:
  /**
   * Initialize the compass module
   *
   * `Returns` - `true` if initialization is successful, `false` otherwise
   */
  bool init();

  /**
   * Read the compass heading
   *
   * `Returns` - heading in degrees `(0~360°)`, clockwise
   */
  uint16_t read(void);

  /**
   * Read raw accelerometer data
   *
   * `Returns` - array of `accelData[3]` (X, Y, Z)
   */
  int16_t* readRawAccel(void);

  /**
   * Read raw gyroscope data
   *
   * `Returns` - array of `gyroData[3]` (X, Y, Z)
   */
  int16_t* readRawGyro(void);

  /**
   * Read raw magnetometer data
   *
   * `Returns` - array of `magData[3]` (X, Y, Z)
   */
  int16_t* readRawMag(void);

  /** Compass coordinate system, adjustable via converter */
  Converter converter;
};

#endif // COMPASS_H