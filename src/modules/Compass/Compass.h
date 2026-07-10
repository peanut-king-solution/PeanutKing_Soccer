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
private:
  uint8_t _address; // I2C address of the compass module
  I2C_Handle _handle; // I2C handle for communication with the compass module

  uint16_t compass = 0; // Variable to store the compass reading
  uint8_t rxBuff[COMPASS_RX_BUFFER_SIZE];  // Buffer for I2C read operations
  uint8_t txBuff[COMPASS_TX_BUFFER_SIZE];  // Buffer for I2C write operations

  // Raw sensor data arrays
  int16_t accelData[3];  // Accelerometer raw data (X, Y, Z)
  int16_t gyroData[3];   // Gyroscope raw data (X, Y, Z)
  int16_t magData[3];    // Magnetometer raw data (X, Y, Z)

public:
  /**
   * Constructor
   * `address` - I2C address of the compass module (default: 0x08)
   */
  Compass(uint8_t address = COMPASS_I2C_ADDRESS);

  /**
   * Initialize the compass module
   * `speed` - I2C bus speed in Hz (default: 400000)
   *
   * `Returns` - `true` if successful, `false` otherwise
   */
  bool init(uint32_t speed = 400000L);

  /**
   * Read the compass heading
   *
   * `Returns` - heading in degrees (`0`~`360`), clockwise
   */
  uint16_t read(void);

  /**
   * Get raw accelerometer data
   *
   * `Returns` - pointer to `accelData[3]` (X, Y, Z)
   */
  int16_t* getAccelerometerRaw(void);

  /**
   * Get raw gyroscope data
   *
   * `Returns` - pointer to `gyroData[3]` (X, Y, Z)
   */
  int16_t* getGyroscopeRaw(void);

  /**
   * Get raw magnetometer data
   *
   * `Returns` - pointer to `magData[3]` (X, Y, Z)
   */
  int16_t* getMagnetometerRaw(void);

  /**
   * Clear the receive buffer
   */
  void rxbufferClear(void);

  Converter converter; // Converter for compass readings

private:
  /**
   * Read raw 6-byte sensor data
   * `reg`      - Register address to read from
   * `dataArr`  - Output array to store data (int16_t[3])
   *
   * `Returns` - pointer to the data array
   */
  int16_t* readRaw6(uint8_t reg, int16_t* dataArr);
};

#endif // COMPASS_H