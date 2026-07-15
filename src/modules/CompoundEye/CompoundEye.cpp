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
  i2cManager.SensorRead(_handle, 0x0D, &val, 1);  // Register 13
  return val;
}

uint8_t CompoundEye::getMaxEyeVal(void)
{
  uint8_t val = 0;
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handle, 0x0C, &val, 1);  // Register 12
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

void CompoundEye::calibrate(float *calData)
{
  uint8_t msg[25] = {IR_CAL};
  uint16_t eyeCal[12];

  for (uint8_t i = 0; i < 12; i++) {
    eyeCal[i] = 4096.0 / calData[i];
    msg[2*i + 1] = eyeCal[i] & 0xff;
    msg[2*i + 2] = eyeCal[i] >> 8;
  }

  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorSend(_handle, msg, 25);
}