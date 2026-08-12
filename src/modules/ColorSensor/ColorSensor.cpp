#include "ColorSensor.h"

I2C_Handle &ColorSensor::getHandle(ColorSensorId sensorNum)
{
  return _handles[sensorNum];
}

ColorSensor::ColorSensor() :
  _handles{    // Initialize I2C handles for 8 color sensors
    I2C_Handle(BusIndex::SW0, COLOR_SENSOR_ADDRESS, 0),
    I2C_Handle(BusIndex::SW1, COLOR_SENSOR_ADDRESS, 0),
    I2C_Handle(BusIndex::SW2, COLOR_SENSOR_ADDRESS, 0),
    I2C_Handle(BusIndex::SW3, COLOR_SENSOR_ADDRESS, 0),
    I2C_Handle(BusIndex::SW4, COLOR_SENSOR_ADDRESS, 0),
    I2C_Handle(BusIndex::SW5, COLOR_SENSOR_ADDRESS, 0),
    I2C_Handle(BusIndex::SW6, COLOR_SENSOR_ADDRESS, 0),
    I2C_Handle(BusIndex::SW7, COLOR_SENSOR_ADDRESS, 0)
  },
  _enabledMask(0x0F), // Default: first four sensors enabled (CL1-CL4)
  _sensorMap{CL1, CL2, CL3, CL4}  // Default: CL1=Front, CL2=Right, CL3=Back, CL4=Left
{
  // Initialize baseline data for all sensors
  for (uint8_t port = CL1; port <= CL8; port++) {
    _baseline[port] = {0, 0, 0, false};
    _isWhite[port] = false;
  }
}

bool ColorSensor::init(void)
{
  // Initialize all enabled sensors and calibrate their baselines
  for (uint8_t port = CL1; port <= CL8; port++) {
    ColorSensorId sensorId = static_cast<ColorSensorId>(port);
    // Skip disabled sensors
    if (!isEnabled(sensorId)) { continue; }
    
    // Calibrate the baseline for this sensor
    calBaseline(sensorId);
  }
  return true;
}

/* =============================================================================
 *                        Sensor Configuration
 * ============================================================================= */

bool ColorSensor::portValidCheck(ColorSensorId sensorNum)
{
  return (sensorNum >= CL1 && sensorNum <= CL8);
}

ColorSensorId ColorSensor::getPortFromPos(SensorPos pos)
{
  // Validate position input
  if (pos < Front || pos > Left) { return ColorSensorMaxCount; } 
  // Return the corresponding ColorSensorId based on the current mapping
  return _sensorMap[pos];
}

void ColorSensor::mapPort(ColorSensorId Front, ColorSensorId Right, ColorSensorId Back, ColorSensorId Left)
{
  if (  // Check if all sensor ports are valid
    !portValidCheck(Front) || 
    !portValidCheck(Right) || 
    !portValidCheck(Back) || 
    !portValidCheck(Left)
  ) {
    return; // Invalid sensor port, do nothing
  }

  // 1. Snapshot old map before overwriting
  ColorSensorId oldMap[4] = { _sensorMap[0], _sensorMap[1], _sensorMap[2], _sensorMap[3] };

  // 2. Update sensor position mapping
  _sensorMap[0] = Front;
  _sensorMap[1] = Right;
  _sensorMap[2] = Back;
  _sensorMap[3] = Left;

  // 3. Old handle cleanup: sensors that were in old map but not in new map
  //    `enable(oldSid, false)` handles unregistering the I2C slot, resetting the
  //    handle, clearing the baseline, and clearing the enable bit.
  for (uint8_t i = 0; i < 4; i++) {
    ColorSensorId oldSid = oldMap[i];

    // Check if this old sensor is still referenced in the new map
    bool stillReferenced = false;
    for (uint8_t j = 0; j < 4; j++) {
      if (_sensorMap[j] == oldSid) { stillReferenced = true; break; }
    }
    if (stillReferenced || !isEnabled(oldSid)) { continue; }

    enable(oldSid, false);
  }

  // 4. Enable and calibrate any new sensor not yet enabled in the map
  //    `enable(sid, true)` registers the I2C slot and hands out a valid handle.
  for (uint8_t i = 0; i < 4; i++) {
    SensorPos pos = static_cast<SensorPos>(i);
    ColorSensorId sid = getPortFromPos(pos);

    // Already handled by init() or a previous mapPort()
    if (isEnabled(sid)) { continue; }
    // Enable the new sensor (registers with I2CManager)
    enable(sid, true);
    calBaseline(sid);
  }
}

/* =============================================================================
 *                        Sensor Enable/Disable
 * ============================================================================= */

void ColorSensor::setEnableMask(uint8_t mask)
{
  // Apply the enable/disable edge on a per-sensor basis so that `enable()`
  // handles I2C registration/unregistration alongside the mask bit.
  // Sensors newly added to the mask get registered; sensors removed get unregistered.
  for (uint8_t port = CL1; port <= CL8; port++) {
    ColorSensorId sensorId = static_cast<ColorSensorId>(port);
    // Determine if this sensor should be enabled based on the mask
    bool wantEnabled = mask & (1 << port);
    // If the desired state differs from the current state, call `enable()` to handle the transition
    if (wantEnabled != isEnabled(sensorId)) {
      enable(sensorId, wantEnabled);
    }
  }
}

void ColorSensor::enable(ColorSensorId sensor, bool enabled)
{
  if (!portValidCheck(sensor)) { return; }  // Invalid sensor port, do nothing

  if (enabled) {
    // Avoid re-enabling an already-enabled sensor
    if (isEnabled(sensor)) return;

    _enabledMask |= (1 << sensor);
  }
  else {
    // Avoid disabling an already-disabled sensor
    if (!isEnabled(sensor)) return;

    // Clear calibration baseline
    _baseline[sensor] = {0, 0, 0, false};
    // Clear white line detection state
    _isWhite[sensor] = false;
    // Disable the sensor in the enable mask
    _enabledMask &= ~(1 << sensor);
  }
}

bool ColorSensor::isEnabled(ColorSensorId sensor) const
{
  return _enabledMask & (1 << sensor);
}

/* =============================================================================
 *                            Read Functions
 * ============================================================================= */

RGBC ColorSensor::readRGBRaw(ColorSensorId sensorNum)
{
  RGBC temp = {0, 0, 0, 0};

  // Return early if the sensor is disabled
  if (!portValidCheck(sensorNum) || !isEnabled(sensorNum))  { return temp; }

  // Read 16 bytes from register 0x02
  // Datasheet order: [127:96] BLUE_RAW [95:64] GREEN_RAW [63:32] RED_RAW [31:0] CLEAR_RAW
  // I2C transmits LSB first, so actual byte order is: CLEAR, RED, GREEN, BLUE
  // Each channel is 4 bytes (32 bits), but sensor uses only lower 16 bits
  I2CManager &i2cManager = I2CManager::getInstance();
  if (i2cManager.SensorRead(_handles[sensorNum], 0x02, _rxBuffer, 16))
  {
    temp.c = (uint32_t)(_rxBuffer[0] | (_rxBuffer[1] << 8) | (_rxBuffer[2] << 16) | (_rxBuffer[3] << 24));  // CLEAR_RAW  [31:0]
    temp.r = (uint32_t)(_rxBuffer[4] | (_rxBuffer[5] << 8) | (_rxBuffer[6] << 16) | (_rxBuffer[7] << 24));  // RED_RAW    [63:32]
    temp.g = (uint32_t)(_rxBuffer[8] | (_rxBuffer[9] << 8) | (_rxBuffer[10] << 16) | (_rxBuffer[11] << 24));  // GREEN_RAW  [95:64]
    temp.b = (uint32_t)(_rxBuffer[12] | (_rxBuffer[13] << 8) | (_rxBuffer[14] << 16) | (_rxBuffer[15] << 24)); // BLUE_RAW   [127:96]
  }

  return temp;
}

RGB ColorSensor::readRGB(ColorSensorId sensorNum)
{
  RGB temp = {0, 0, 0};

  // Return early if the sensor is disabled
  if (!portValidCheck(sensorNum) || !isEnabled(sensorNum))  { return temp; }
  
  I2CManager &i2cManager = I2CManager::getInstance();
  for (int i = 0; i < _maxRetry; i++) {
    // Read 3 bytes from register 0x08 (calculated RGB: R=8, G=8, B=8)
    if (i2cManager.SensorRead(_handles[sensorNum], 0x08, _rxBuffer, 3))
    {
      temp.r = _rxBuffer[0];
      temp.g = _rxBuffer[1];
      temp.b = _rxBuffer[2];

      if (temp.r <= 255 && temp.g <= 255 && temp.b <= 255) {
        return temp; // Successful read within valid range
      }
    }
    // If the read failed or values are out of range, retry up to 5 times
  }
  return temp;
}

HSL ColorSensor::readHSL(ColorSensorId sensorNum)
{
  HSL temp = {0, 0, 0};

  // Return early if the sensor is disabled
  if (!portValidCheck(sensorNum) || !isEnabled(sensorNum))  { return temp; }

  // Read 4 bytes from register 0x03 (HSL: H=16, S=8, L=8)
  I2CManager &i2cManager = I2CManager::getInstance();
  for (int i = 0; i < _maxRetry; i++) {
    if (i2cManager.SensorRead(_handles[sensorNum], 0x03, _rxBuffer, 4))
    {
      temp.h = (uint16_t)(_rxBuffer[0] | (_rxBuffer[1] << 8));
      temp.s = _rxBuffer[2];
      temp.l = _rxBuffer[3];

      if (temp.h < 360 && temp.s <= 100 && temp.l <= 100) {
        return temp; // Successful read within valid range
      }
    }
    // If the read failed or values are out of range, retry up to 5 times
  }
  return temp;
}

/* =============================================================================
 *                             LED Control
 * ============================================================================= */

bool ColorSensor::whiteLedOn(ColorSensorId sensorNum)
{
  if (!portValidCheck(sensorNum) || !isEnabled(sensorNum))  { return false; }

  // Write register address 0x04 to turn on white LED
  uint8_t reg = 0x04;
  I2CManager &i2cManager = I2CManager::getInstance();
  return i2cManager.SensorSend(_handles[sensorNum], &reg, 1);
}

bool ColorSensor::whiteLedOff(ColorSensorId sensorNum)
{
  if (!portValidCheck(sensorNum) || !isEnabled(sensorNum))  { return false; }

  // Write register address 0x05 to turn off white LED
  uint8_t reg = 0x05;
  I2CManager &i2cManager = I2CManager::getInstance();
  return i2cManager.SensorSend(_handles[sensorNum], &reg, 1);
}

bool ColorSensor::rgbwLedOn(ColorSensorId sensorNum)
{
  if (!portValidCheck(sensorNum) || !isEnabled(sensorNum))  { return false; }

  // Write register address 0x06 to turn on RGBW LED
  uint8_t reg = 0x06;
  I2CManager &i2cManager = I2CManager::getInstance();
  return i2cManager.SensorSend(_handles[sensorNum], &reg, 1);
}

bool ColorSensor::rgbwLedOff(ColorSensorId sensorNum)
{
  if (!portValidCheck(sensorNum) || !isEnabled(sensorNum))  { return false; }

  // Write register address 0x07 to turn off RGBW LED
  uint8_t reg = 0x07;
  I2CManager &i2cManager = I2CManager::getInstance();
  return i2cManager.SensorSend(_handles[sensorNum], &reg, 1);
}

/* =============================================================================
 *                        White Line Detection (Plan A: Baseline)
 * ============================================================================= */

bool ColorSensor::calBaseline(ColorSensorId n, uint8_t samples)
{
  // Return early if the sensor port is invalid or disabled
  if (!portValidCheck(n) || !isEnabled(n)) { return false; }

  if (samples == 0) { samples = 10; } // Default to 10 samples if not specified

  uint32_t sumH = 0, sumS = 0, sumL = 0;
  uint8_t  count = 0;

  // Collect multiple samples to average the baseline values
  for (uint8_t i = 0; i < samples; i++) {
    HSL hsl = readHSL(n);
    sumH += hsl.h; sumS += hsl.s; sumL += hsl.l;
    count++;
    delay(10);
  }

  _baseline[n].greenHue   = (uint16_t)(sumH / count);
  _baseline[n].greenSat   = (uint8_t) (sumS / count);
  _baseline[n].greenLight = (uint8_t) (sumL / count);
  _baseline[n].calibrated = true;
  return true;
}

GreenBaseline ColorSensor::getBaseline(ColorSensorId n) const
{
  GreenBaseline temp = {0, 0, 0, false};
  // Return early if the sensor port is invalid or disabled
  if (!portValidCheck(n) || !isEnabled(n)) { return temp; }
  return _baseline[n];
}

bool ColorSensor::isWhiteLine(ColorSensorId n)
{
  if (!portValidCheck(n) || !isEnabled(n) || !_baseline[n].calibrated) return false;

  HSL hsl = readHSL(n);

  // 3D detection: Light + Sat + Hue, 2/3 vote
  // uint8_t lightMargin = _baseline[n].greenLight / 5;
  // uint8_t satThresh   = (_baseline[n].greenSat * 80) / 100;
  // uint16_t hueDiff    = abs((int)hsl.h - (int)_baseline[n].greenHue);

  // bool lightCheck = hsl.l > _baseline[n].greenLight + lightMargin;
  // bool satCheck   = hsl.s < satThresh;
  // bool hueCheck   = hueDiff > 30;

  // _isWhite[n] = (lightCheck + satCheck + hueCheck) >= 2;

  uint16_t hueMargin = _baseline[n].greenHue / 8;
  bool hueCheck = hsl.h > (_baseline[n].greenHue + hueMargin);
  _isWhite[n] = hueCheck;

  return _isWhite[n];
}
