#include "ColorSensor.h"

I2C_Handle &ColorSensor::getHandle(CLR_SENSOR_ID sensorNum)
{
  return _handles[sensorNum];
}

ColorSensor::ColorSensor() :
  _handles{    // Initialize I2C handles for 8 color sensors
    I2C_Handle(BusIndex::SW0, 0, 0),
    I2C_Handle(BusIndex::SW1, 0, 0),
    I2C_Handle(BusIndex::SW2, 0, 0),
    I2C_Handle(BusIndex::SW3, 0, 0),
    I2C_Handle(BusIndex::SW4, 0, 0),
    I2C_Handle(BusIndex::SW5, 0, 0),
    I2C_Handle(BusIndex::SW6, 0, 0),
    I2C_Handle(BusIndex::SW7, 0, 0)
  },
  _enabledMask(0x0F) // Default: first four sensors enabled (CL1-CL4)
{
}

bool ColorSensor::init(void)
{
  I2CManager &i2cManager = I2CManager::getInstance();
  // Register only enabled sensors with I2CManager
  for (uint8_t i = 0; i < 8; i++) {
    // Skip disabled sensors
    if (!(_enabledMask & (1 << i))) { continue; }

    // Register the sensor with I2CManager and store the handle
    _handles[i] = i2cManager.RegisterDevice(
      static_cast<BusIndex>(i),
      COLOR_SENSOR_ADDRESS,
      100000
    );

    // Check if the handle is valid after registration
    if (!_handles[i].isValid()) { return false; }
  }
  return true;
}


/* =============================================================================
 *                        Sensor Enable/Disable
 * ============================================================================= */

void ColorSensor::setEnabled(uint8_t mask)
{
  _enabledMask = mask;
}

void ColorSensor::enableSensor(CLR_SENSOR_ID sensor, bool enabled)
{
  if (enabled)  {
    _enabledMask |= (1 << sensor);
  }
  else {
    _enabledMask &= ~(1 << sensor);
  }
}

bool ColorSensor::isEnabled(CLR_SENSOR_ID sensor) const
{
  return _enabledMask & (1 << sensor);
}

/* =============================================================================
 *                             Data Reading
 * ============================================================================= */

uint8_t ColorSensor::readColor(CLR_SENSOR_ID sensorNum)
{
  // Return early if the sensor is disabled
  if (!isEnabled(sensorNum))  { return 0; }

  uint8_t val = 0;

  // Read single byte from register 0x01 (color index)
  I2CManager &i2cManager = I2CManager::getInstance();
  i2cManager.SensorRead(_handles[sensorNum], 0x01, &val, 1);

  return val;
}

rgbc_t ColorSensor::readRGBRaw(CLR_SENSOR_ID sensorNum)
{
  rgbc_t temp = {0, 0, 0, 0};

  // Return early if the sensor is disabled
  if (!isEnabled(sensorNum))  { return temp; }

  // Read 16 bytes from register 0x02
  // Datasheet order: [127:96] BLUE_RAW [95:64] GREEN_RAW [63:32] RED_RAW [31:0] CLEAR_RAW
  // I2C transmits LSB first, so actual byte order is: CLEAR, RED, GREEN, BLUE
  // Each channel is 4 bytes (32 bits), but sensor uses only lower 16 bits
  I2CManager &i2cManager = I2CManager::getInstance();
  if (i2cManager.SensorRead(_handles[sensorNum], 0x02, _rxBuffer, 16))
  {
    temp.c = (uint16_t)(_rxBuffer[0] | (_rxBuffer[1] << 8));  // CLEAR_RAW  [31:0]
    temp.r = (uint16_t)(_rxBuffer[4] | (_rxBuffer[5] << 8));  // RED_RAW    [63:32]
    temp.g = (uint16_t)(_rxBuffer[8] | (_rxBuffer[9] << 8));  // GREEN_RAW  [95:64]
    temp.b = (uint16_t)(_rxBuffer[12] | (_rxBuffer[13] << 8)); // BLUE_RAW   [127:96]
  }

  return temp;
}

rgb_t ColorSensor::readRGB(CLR_SENSOR_ID sensorNum)
{
  rgb_t temp = {0, 0, 0};

  // Return early if the sensor is disabled
  if (!isEnabled(sensorNum))  { return temp; }

  // Read 3 bytes from register 0x08 (calculated RGB: R=8, G=8, B=8)
  I2CManager &i2cManager = I2CManager::getInstance();
  if (i2cManager.SensorRead(_handles[sensorNum], 0x08, _rxBuffer, 3))
  {
    temp.r = _rxBuffer[0];
    temp.g = _rxBuffer[1];
    temp.b = _rxBuffer[2];
  }

  return temp;
}

hsl_t ColorSensor::readHSL(CLR_SENSOR_ID sensorNum)
{
  hsl_t temp = {0, 0, 0};

  // Return early if the sensor is disabled
  if (!isEnabled(sensorNum))  { return temp; }

  // Read 4 bytes from register 0x03 (HSL: H=16, S=8, L=8)
  I2CManager &i2cManager = I2CManager::getInstance();
  if (i2cManager.SensorRead(_handles[sensorNum], 0x03, _rxBuffer, 4))
  {
    temp.h = (uint16_t)(_rxBuffer[0] | (_rxBuffer[1] << 8));
    temp.s = _rxBuffer[2];
    temp.l = _rxBuffer[3];
  }

  return temp;
}

/* =============================================================================
 *                             LED Control
 * ============================================================================= */

bool ColorSensor::whiteLedOn(CLR_SENSOR_ID sensorNum)
{
  if (!isEnabled(sensorNum))  { return false; }

  // Write register address 0x04 to turn on white LED
  uint8_t reg = 0x04;
  I2CManager &i2cManager = I2CManager::getInstance();
  return i2cManager.SensorSend(_handles[sensorNum], &reg, 1);
}

bool ColorSensor::whiteLedOff(CLR_SENSOR_ID sensorNum)
{
  if (!isEnabled(sensorNum))  { return false; }

  // Write register address 0x05 to turn off white LED
  uint8_t reg = 0x05;
  I2CManager &i2cManager = I2CManager::getInstance();
  return i2cManager.SensorSend(_handles[sensorNum], &reg, 1);
}

bool ColorSensor::rgbwLedOn(CLR_SENSOR_ID sensorNum)
{
  if (!isEnabled(sensorNum))  { return false; }

  // Write register address 0x06 to turn on RGBW LED
  uint8_t reg = 0x06;
  I2CManager &i2cManager = I2CManager::getInstance();
  return i2cManager.SensorSend(_handles[sensorNum], &reg, 1);
}

bool ColorSensor::rgbwLedOff(CLR_SENSOR_ID sensorNum)
{
  if (!isEnabled(sensorNum))  { return false; }

  // Write register address 0x07 to turn off RGBW LED
  uint8_t reg = 0x07;
  I2CManager &i2cManager = I2CManager::getInstance();
  return i2cManager.SensorSend(_handles[sensorNum], &reg, 1);
}
