#ifndef HW_I2CMASTER_H
#define HW_I2CMASTER_H

#define I2C_DEFAULT_SPEED 400000 // Default I2C speed in `Hz` (`400kHz`)
#define I2C_MAX_BUFFER_SIZE 64   // Maximum buffer size for I2C transactions

#include <Arduino.h>

// If user explicitly requests Wire library via define or inclusion:
#if defined(USE_WIRE_H)
  // User explicitly chose Wire.h
  #include <Wire.h>
  #define HW_I2C_USE_WIRE_H
  #pragma message "Use Wire.h"
#elif __has_include(<Wire.h>)
  // Auto-detect: Wire.h is available
  #include <Wire.h>
  #define HW_I2C_USE_WIRE_H
  #pragma message "Auto-detected Wire.h"
#else
  // Fall back to IICIT
  #include "IICIT.h"
  #pragma message "Using IICIT"
#endif

/**
 * Hardware I2C master wrapper class
 * Supports both Wire.h and IICIT backends
 * - Define USE_WIRE_H or include <Wire.h> before this header to use Wire
 * - Otherwise auto-detects Wire.h availability via __has_include
 * - Falls back to IICIT when Wire.h is not available
 */
class hwI2CMaster
{
public:
  // Constructor
  hwI2CMaster();

  /**
   * Initialize the hardware I2C with specified speed
   * `speed` - I2C speed in `Hz` (default: `400000`)
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

  #ifndef HW_I2C_USE_WIRE_H
  /**
   * I2C receive complete callback (IICIT only)
   * `status` - Status of the I2C operation
   *
   * `Returns` - The status value
   */
  static IICIT::status_t rxCpltCallback(const IICIT::status_t status);
#endif

private:
  uint32_t defaultSpeed;

#ifndef HW_I2C_USE_WIRE_H
  /**
   * Convert speed in `Hz` to `IICIT::Speed` enum (IICIT only)
   * `speed` - Speed in `Hz`
   *
   * `Returns` - `IICIT::Speed` enum value
   */
  IICIT::Speed speedToIICIT(uint32_t speed) const;
#endif
};

#endif // HW_I2CMASTER_H