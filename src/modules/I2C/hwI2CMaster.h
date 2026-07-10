#ifndef HW_I2CMASTER_H
#define HW_I2CMASTER_H

#define I2C_DEFAULT_SPEED 400000 // Default I2C speed in Hz (400kHz)
#define I2C_MAX_BUFFER_SIZE 64   // Maximum buffer size for I2C transactions

#include <Arduino.h>
#include "IICIT.h"

/**
 * Hardware I2C master wrapper class
 * Uses IICIT library for low-level I2C communication
 */
class hwI2CMaster
{
public:
  /**
   * Constructor
   */
  hwI2CMaster();

  /**
   * Initialize the hardware I2C with specified speed
   * `speed` - I2C speed in Hz (default: 400000)
   *
   * `Returns` - `true` if successful, `false` otherwise
   */
  bool init(uint32_t speed = I2C_DEFAULT_SPEED);

  /**
   * Read data from a device at the specified address
   * `deviceAddress` - I2C device address
   * `rxBuffer`      - Buffer to store received data
   * `length`        - Number of bytes to read
   *
   * `Returns` - `true` if successful, `false` otherwise
   */
  bool read(uint8_t deviceAddress, uint8_t *rxBuffer, uint8_t length);

  /**
   * Write data to a device at the specified address
   * `deviceAddress` - I2C device address
   * `txBuffer`      - Buffer containing data to send
   * `length`        - Number of bytes to send
   *
   * `Returns` - `true` if successful, `false` otherwise
   */
  bool write(uint8_t deviceAddress, const uint8_t *txBuffer, uint8_t length);

  /**
   * Read data from a specific register
   * `deviceAddress` - I2C device address
   * `reg`           - Register address to read from
   * `rxBuffer`      - Buffer to store received data
   * `length`        - Number of bytes to read
   *
   * `Returns` - `true` if successful, `false` otherwise
   */
  bool readReg(uint8_t deviceAddress, uint8_t reg, uint8_t *rxBuffer, uint8_t length);

  /**
   * Write data to a specific register
   * `deviceAddress` - I2C device address
   * `reg`           - Register address to write to
   * `txBuffer`      - Buffer containing data to send
   * `length`        - Number of bytes to send
   *
   * `Returns` - `true` if successful, `false` otherwise
   */
  bool writeReg(uint8_t deviceAddress, uint8_t reg, const uint8_t *txBuffer, uint8_t length);

private:
  uint32_t defaultSpeed;

  /**
   * Convert speed in Hz to IICIT::Speed enum
   * `speed` - Speed in Hz
   *
   * `Returns` - IICIT::Speed enum value
   */
  IICIT::Speed speedToIICIT(uint32_t speed) const;
};

#endif // HW_I2CMASTER_H