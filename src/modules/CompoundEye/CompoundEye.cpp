#include "CompoundEye.h"

CompoundEye::CompoundEye()
  : _handle(BusIndex::HW, COMPOUND_EYE_I2C_ADDRESS, 400000)  // I2C speed: 400kHz
{
}

bool CompoundEye::init(void)
{
  // Return false if the handle is invalid
  if (!_handle.isValid()) { return false; }

  this->converter.reset();  // Reset the converter to default state

  return true;
}

bool CompoundEye::indexValidCheck(EyeId eyeIndex) const
{
  return (eyeIndex >= Eye0) && (eyeIndex <= Eye11);
}

uint8_t* CompoundEye::readAll(void)
{
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, IR_RAW, _eye, 12);
  return _eye;
}

EyeId CompoundEye::readMaxEye(void)
{
  uint8_t val = 0;
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, IR_MAX_IDX, &val, 1);  // Register 13
  return (EyeId)val;
}

uint8_t CompoundEye::readMaxEyeVal(void)
{
  uint8_t val = 0;
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, IR_MAX, &val, 1);  // Register 12
  return val;
}

uint8_t CompoundEye::readEyeVal(EyeId eyeIndex)
{
  if (!indexValidCheck(eyeIndex)) return 0;

  uint8_t val = 0;
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, (uint8_t)eyeIndex, &val, 1);
  return val;
}

uint16_t CompoundEye::readAngle(void)
{
  uint8_t val = 0;
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, IR_ANGLE, &val, 1);  // 1 byte from firmware
  float angle = (float)((uint16_t)val * 2);  // Firmware /2, multiply back to 0-360
  angle = converter.convert(angle);  // Apply coordinate conversion
  return (uint16_t)angle;
}

uint8_t CompoundEye::readMode(void)
{
  uint8_t val = 0;
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, IR_MODE, &val, 1);  // Register 15
  return val;
}