#ifndef HW_I2CMASTER_H
#define HW_I2CMASTER_H

#define I2C_DEFAULT_SPEED 400000 // Default I2C speed in Hz (400kHz)
#define I2C_MAX_BUFFER_SIZE 64   // Maximum buffer size for I2C transactions

#include <Arduino.h>
#include "IICIT.h"

class hwI2CMaster
{
public:
  // Constructor
  hwI2CMaster();

  // Initialize the hardware I2C with specified speed
  bool init(uint32_t speed = I2C_DEFAULT_SPEED);

  // Read data from a device at the specified address
  bool read(uint8_t deviceAddress, uint8_t *rxBuffer, uint8_t length);
  // Write data to a device at the specified address
  bool write(uint8_t deviceAddress, const uint8_t *txBuffer, uint8_t length);

  bool readReg(uint8_t deviceAddress, uint8_t reg, uint8_t *rxBuffer, uint8_t length);
  bool writeReg(uint8_t deviceAddress, uint8_t reg, const uint8_t *txBuffer, uint8_t length);

private:
  // Store the default speed for I2C communication
  uint32_t defaultSpeed;
  // Convert speed in Hz to IICIT::Speed enum
  IICIT::Speed speedToIICIT(uint32_t speed) const;
};

#endif // HW_I2CMASTER_H