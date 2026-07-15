#include "CompoundEye.h"

CompoundEye::CompoundEye(uint8_t address)
  : _address(address), _handle(BusIndex::HW, 0x00, 0)
{
  // Initialize eye buffer to zero
  for (uint8_t i = 0; i < 12; i++) {
    _eye[i] = 0;
  }
}

bool CompoundEye::init(void)
{
  I2CManager &i2cManager = I2CManager::getInstance();
  _handle = i2cManager.RegisterDevice(BusIndex::HW, _address, 400000);

  return _handle.isValid();
}

uint8_t* CompoundEye::readAll(void)
{
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, IR_RAW, _eye, 12);
  return _eye;
}

uint8_t CompoundEye::getMaxEye(void)
{
  uint8_t val = 0;
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, IR_MAX_IDX, &val, 1);  // Register 13
  return val;
}

uint8_t CompoundEye::getMaxEyeVal(void)
{
  uint8_t val = 0;
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, IR_MAX, &val, 1);  // Register 12
  return val;
}

uint8_t CompoundEye::getEyeVal(uint8_t n)
{
  if (n >= 12) return 0;

  uint8_t val = 0;
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, n, &val, 1);
  return val;
}

uint16_t CompoundEye::getAngle(void)
{
  uint8_t val = 0;
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, IR_ANGLE, &val, 1);  // 1 byte from firmware
  return (uint16_t)val * 2;  // Firmware /2, multiply back to 0-360
}

uint8_t CompoundEye::getMode(void)
{
  uint8_t val = 0;
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, IR_MODE, &val, 1);  // Register 15
  return val;
}