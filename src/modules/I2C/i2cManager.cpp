#include "i2cManager.h"

I2CManager::I2CManager() : 
  swiic{  // Software I2C pin (sda, scl) & pullup init
    SlowSoftI2CMaster(29, 30, 1),
    SlowSoftI2CMaster(31, 32, 1),
    SlowSoftI2CMaster(33, 34, 1),
    SlowSoftI2CMaster(35, 36, 1),
    SlowSoftI2CMaster(37, 38, 1),
    SlowSoftI2CMaster(39, 40, 1),
    SlowSoftI2CMaster(41, 42, 1),
    SlowSoftI2CMaster(43, 44, 1)}
{
}

bool I2CManager::init(void)
{
  // Initialize software I2C instances
  for (uint8_t i = 0; i < 8; i++) {
    swiic[i].i2c_init();
  }

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

    // Start I2C transaction
    if (!swiic[busIdx].i2c_start(addr | I2C_WRITE)) return false;
    
    // Write the register address to the device
    swiic[busIdx].i2c_write(reg);
    // restart for reading
    swiic[busIdx].i2c_rep_start(addr | I2C_READ);

    // Read data bytes (send NAK on last byte)
    for (uint8_t i = 0; i < length; i++) {
      rxBuffer[i] = swiic[busIdx].i2c_read(i == length - 1);
    }

    // Stop I2C transaction
    swiic[busIdx].i2c_stop();
    return true;
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

  uint8_t busIdx = static_cast<uint8_t>(handle.busIndex);

  // Use software I2C for writing (SW0-SW7)
  if (busIdx < 8) {
    uint8_t addr = handle.deviceAddress << 1;

    // Start I2C transaction
    if (!swiic[busIdx].i2c_start(addr | I2C_WRITE)) return false;

    // Write register address
    swiic[busIdx].i2c_write(txBuffer[0]);

    // Write data bytes (skip first byte which is register address)
    for (uint8_t i = 1; i < length; i++) {
      if (!swiic[busIdx].i2c_write(txBuffer[i])) {
        swiic[busIdx].i2c_stop();
        return false;
      }
    }

    // Stop I2C transaction
    swiic[busIdx].i2c_stop();
    return true;
  }
  // Use hardware I2C for writing
  else {
    // Write the data from the buffer to the device
    return hwiic.write(handle.deviceAddress, txBuffer, length);
  }
}