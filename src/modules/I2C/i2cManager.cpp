#include "i2cManager.h"

I2CManager *I2CManager = &I2CManager::getInstance();

I2CManager::I2CManager()
  // : swiic{  // software I2C instances for multiple I2C buses
  //   SlowSoftI2CMaster(29, 30, 1),
  //   SlowSoftI2CMaster(31, 32, 1),
  //   SlowSoftI2CMaster(33, 34, 1),
  //   SlowSoftI2CMaster(35, 36, 1),
  //   SlowSoftI2CMaster(37, 38, 1),
  //   SlowSoftI2CMaster(39, 40, 1),
  //   SlowSoftI2CMaster(41, 42, 1),
  //   SlowSoftI2CMaster(43, 44, 1)}
{
}

bool I2CManager::init(void)
{
  // (software I2C is initialized in PeanutKingSoccerV4::init()
  // later move the software I2C initialization here

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

  // Select the appropriate I2C instance based on the bus index
  // Use software I2C for reading
  if (handle.busIndex < BusIndex::HW) {
    return false; // Software I2C read not yet implemented
  }
  // Use hardware I2C for reading
  else {
    return hwiic.readReg(handle.deviceAddress, reg, rxBuffer, length); // Read from the device
  }
}

bool I2CManager::SensorSend(const I2C_Handle &handle, const uint8_t *txBuffer, uint8_t length)
{
  // Validate the I2C handle and buffer parameters
  if (!handle.isValid() || txBuffer == nullptr || length == 0)
  {
    return false; // Invalid handle or buffer
  }

  // Select the appropriate I2C instance based on the bus index
  // Use software I2C for writing
  if (handle.busIndex < BusIndex::HW) {
    return false; // Software I2C write not yet implemented
  }
  // Use hardware I2C for writing
  else {
    // Write the data from the buffer to the device
    return hwiic.write(handle.deviceAddress, txBuffer, length);
  }
}