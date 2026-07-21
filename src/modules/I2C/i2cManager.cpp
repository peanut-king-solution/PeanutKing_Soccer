#include "i2cManager.h"

I2CManager::I2CManager() :
  swiic{  // Software I2C pin (scl, sda) init
    SoftI2cMaster(30, 29),
    SoftI2cMaster(32, 31),
    SoftI2cMaster(34, 33),
    SoftI2cMaster(36, 35),
    SoftI2cMaster(38, 37),
    SoftI2cMaster(40, 39),
    SoftI2cMaster(42, 41),
    SoftI2cMaster(44, 43)
  }
{
}

bool I2CManager::init(void)
{
  // Software I2C instances are initialized in constructor via begin()
  // No additional init needed for SoftI2cMaster

  // Initialize the hardware I2C instance
  if (!hwiic.init()) {
    return false; // Return false if hardware I2C instance fails to initialize
  }
  return true;
}

I2C_Handle I2CManager::RegisterDevice(BusIndex busIndex, uint8_t deviceAddress, uint32_t speed)
{
  // Validate the bus index and device address
  if (busIndex > BusIndex::HW || deviceAddress > 0x7F || deviceAddress == 0x00)
  {
    return I2C_Handle(BusIndex::HW, 0x00, 0); // Return an invalid handle
  }
  return I2C_Handle(busIndex, deviceAddress, speed); // Return a valid handle
}

bool I2CManager::SensorRead(const I2C_Handle &handle, uint8_t reg, uint8_t *rxBuffer, uint8_t length)
{
  // Validate the I2C handle and buffer parameters
  if (!handle.isValid() || rxBuffer == nullptr || length == 0)
  {
    return false; // Invalid handle or buffer
  }

  // Determine the bus index for the I2C operation
  uint8_t busIdx = static_cast<uint8_t>(handle.busIndex);
  // Use software I2C for reading (SW0-SW7)
  if (busIdx < 8) {
    uint8_t addr = handle.deviceAddress << 1;

    // START + write device address + write register address
    swiic[busIdx].start();
    if (!swiic[busIdx].write(addr | I2C_WRITE)) {
      swiic[busIdx].stop();
      return false;
    }
    swiic[busIdx].write(reg);

    // Repeated START + write device address (read mode)
    swiic[busIdx].start();
    if (!swiic[busIdx].write(addr | I2C_READ)) {
      swiic[busIdx].stop();
      return false;
    }

    // Read data bytes (send NACK on last byte)
    for (uint8_t i = 0; i < length; i++) {
      rxBuffer[i] = swiic[busIdx].read(i == length - 1);
    }

    // STOP
    swiic[busIdx].stop();
    return true;
  }
  // Use hardware I2C for reading
  else {
    return hwiic.readReg(handle.deviceAddress, reg, rxBuffer, length);
  }
}

bool I2CManager::SensorSend(const I2C_Handle &handle, const uint8_t *txBuffer, uint8_t length)
{
  // Validate the I2C handle and buffer parameters
  if (!handle.isValid() || txBuffer == nullptr || length == 0)
  {
    return false; // Invalid handle or buffer
  }

  uint8_t busIdx = static_cast<uint8_t>(handle.busIndex);

  // Use software I2C for writing (SW0-SW7)
  if (busIdx < 8) {
    uint8_t addr = handle.deviceAddress << 1;

    // START + write device address + write data bytes
    swiic[busIdx].start();
    if (!swiic[busIdx].write(addr | I2C_WRITE)) {
      swiic[busIdx].stop();
      return false;
    }
    for (uint8_t i = 0; i < length; i++) {
      if (!swiic[busIdx].write(txBuffer[i])) {
        swiic[busIdx].stop();
        return false;
      }
    }

    // STOP
    swiic[busIdx].stop();
    return true;
  }
  // Use hardware I2C for writing
  else {
    // Write the data from the buffer to the device
    return hwiic.write(handle.deviceAddress, txBuffer, length);
  }
}