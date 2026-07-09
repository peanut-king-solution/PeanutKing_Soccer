#ifndef I2CMANAGER_H
#define I2CMANAGER_H

#include <Arduino.h>

#include "SlowSoftI2CMaster.h"
#include "hwI2CMaster.h"

enum class BusIndex : uint8_t
{
  SW0 = 0,
  SW1 = 1,
  SW2 = 2,
  SW3 = 3,
  SW4 = 4,
  SW5 = 5,
  SW6 = 6,
  SW7 = 7,
  HW  = 8
};

struct I2C_Handle
{
  BusIndex busIndex;     // Index of I2C bus (0-7 for software, 8 for hardware)
  uint8_t deviceAddress; // I2C device address
  uint32_t speed;        // I2C speed in Hz

  // Constructor to initialize the I2C handle
  I2C_Handle(BusIndex index, uint8_t address, uint32_t i2cSpeed)
      : busIndex(index), deviceAddress(address), speed(i2cSpeed) {}

  // Check if the I2C handle is valid
  bool isValid() const
  {
    return (busIndex <= BusIndex::HW) && (deviceAddress <= 0x7F) && (deviceAddress != 0x00);
  }
};

class I2CManager
{
private:
  I2CManager();                                       // Private constructor for singleton pattern
  I2CManager(const I2CManager &) = delete;            // Delete copy constructor
  I2CManager &operator=(const I2CManager &) = delete; // Delete assignment operator

  // software I2C instances for multiple I2C buses
  // SlowSoftI2CMaster swiic[8];
  // hardware I2C instance
  hwI2CMaster hwiic;

  uint8_t regBuf[1]; // Buffer for register address

public:
  // Get the singleton instance of I2CManager
  static I2CManager &getInstance()
  {
    static I2CManager instance;
    return instance;
  }

  bool init(void); // Initialize all I2C instances

  I2C_Handle RegisterDevice(BusIndex busIndex, uint8_t deviceAddress, uint32_t speed);
  bool SensorRead(const I2C_Handle &handle, uint8_t reg, uint8_t *rxBuffer, uint8_t length);
  bool SensorSend(const I2C_Handle &handle, const uint8_t *txBuffer, uint8_t length);
};

#endif // I2CMANAGER_H