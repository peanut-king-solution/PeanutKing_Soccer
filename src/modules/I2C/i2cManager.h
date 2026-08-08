#ifndef I2CMANAGER_H
#define I2CMANAGER_H

#include <Arduino.h>

#include "software/SoftI2cMaster.h"
#include "hardware/hwI2CMaster.h"

/**
 * I2C bus index enumeration
 * `SW0`-`SW7` - Software I2C buses (0-7)
 * `HW` - Hardware I2C bus (8)
 */
enum class BusIndex : uint8_t
{
  SW0 = 0, SW1, SW2, SW3, SW4, SW5, SW6, SW7, HW, INVALID = 9
};

/**
 * Check if a bus index is within the valid range (`SW0`-`HW`)
 * `busIndex` - Bus index to validate
 *
 * `Returns` - `true` if valid, `false` otherwise
 */
inline bool isValidBusIndex(BusIndex busIndex)
{
  return (busIndex >= BusIndex::SW0) && (busIndex <= BusIndex::HW) && (busIndex != BusIndex::INVALID);
}

/**
 * Check if a device address is within the valid I2C range (`0x01`-`0x7F`)
 * `deviceAddress` - Device address to validate
 *
 * `Returns` - `true` if valid, `false` otherwise
 */
inline bool isValidDeviceAddress(uint8_t deviceAddress)
{
  return (deviceAddress != 0x00) && (deviceAddress <= 0x7F);
}

/**
 * I2C device handle structure
 * `busIndex`      - Index of I2C bus (`0-7` for software, `8` for hardware)
 * `deviceAddress` - I2C device address
 * `speed`         - I2C speed in `Hz`
 */
struct I2C_Handle
{
  BusIndex busIndex;
  uint8_t deviceAddress;
  uint32_t speed;

  // Default Constructor
  I2C_Handle() : busIndex(BusIndex::INVALID), deviceAddress(0), speed(0) {}

  /**
   * Constructor to initialize the I2C handle
   * `index`    - Bus index
   * `address`  - Device address
   * `i2cSpeed` - I2C speed in `Hz`
   */
  I2C_Handle(BusIndex index, uint8_t address, uint32_t i2cSpeed)
    : busIndex(index), deviceAddress(address), speed(i2cSpeed) 
  {
    if (!isValidBusIndex(busIndex)) {
      busIndex = BusIndex::INVALID; // Mark as invalid if bus index is out of range
    }
    if (!isValidDeviceAddress(deviceAddress)) {
      deviceAddress = 0; // Mark as invalid if device address is out of range
    }
  }

  /**
   * Check if the I2C handle is valid
   *
   * `Returns` - `true` if valid, `false` otherwise
   */
  bool isValid() const
  {
    return isValidBusIndex(busIndex) && isValidDeviceAddress(deviceAddress);
  }
};

/**
 * I2CManager class for managing I2C devices and operations
 */
class I2CManager
{
private:
  I2CManager();
  I2CManager(const I2CManager &) = delete;
  I2CManager &operator=(const I2CManager &) = delete;

  SoftI2cMaster swiic[8];   // Software I2C instances for color sensors
  hwI2CMaster hwiic;        // Hardware I2C master instance

public:
  /**
   * Get the singleton instance of I2CManager
   *
   * `Returns` - reference to the singleton instance
   */
  static I2CManager &getInstance()
  {
    static I2CManager instance;
    return instance;
  }

  /**
   * Initialize all I2C instances
   *
   * `Returns` - `true` if successful, `false` otherwise
   */
  bool init(void);

  /**
   * Read data from I2C device
   * `handle`   - I2C handle from RegisterDevice()
   * `reg`      - Register address to read from
   * `rxBuffer` - Buffer to store received data
   * `length`   - Number of bytes to read
   *
   * `Returns` - `true` if successful, `false` otherwise
   */
  bool SensorRead(const I2C_Handle &handle, uint8_t reg, uint8_t *rxBuffer, uint8_t length);

  /**
   * Send data to I2C device
   * `handle`   - I2C handle from RegisterDevice()
   * `txBuffer` - Buffer containing data to send
   * `length`   - Number of bytes to send
   *
   * `Returns` - `true` if successful, `false` otherwise
   */
  bool SensorSend(const I2C_Handle &handle, const uint8_t *txBuffer, uint8_t length);
};

#endif // I2CMANAGER_H