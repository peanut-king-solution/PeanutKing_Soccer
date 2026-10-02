#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#define COLOR_SENSOR_ADDRESS 0x11 // Color sensor I2C address

#include <Arduino.h>
#include "PeanutKingDef.h"
#include "modules/I2C/i2cManager.h"

// Color sensor indices (CL1-CL8)
enum ColorSensorId : uint8_t {
  CL1 = 0, CL2, CL3, CL4, CL5, CL6, CL7, CL8 = 7,
  ColorSensorMaxCount = 8 // Total number of color sensors
};

/**
 * RGBC raw data structure (register 0x02)
 * Raw photodiode spectral responsivity data (0-65535 each)
 */
struct RGBC {
  uint32_t r;  // Red raw
  uint32_t g;  // Green raw
  uint32_t b;  // Blue raw
  uint32_t c;  // Clear raw
};

/**
 * RGB calculated value structure (register 0x08)
 * Normalized RGB values (0-255 each)
 */
struct RGB {
  uint16_t r;  // Red value
  uint16_t g;  // Green value
  uint16_t b;  // Blue value
};

/**
 * HSL value structure (register 0x03)
 * Hue-Saturation-Lightness values
 */
struct HSL {
  uint16_t h;  // Hue value (0-360)
  uint8_t  s;  // Saturation value (0-100)
  uint8_t  l;  // Lightness value (0-100)
};

// Calibrated baseline of green values for white line detection
struct GreenBaseline {
  uint16_t greenHue;   // Average green field hue
  uint8_t  greenLight; // Average green field lightness
  uint8_t  greenSat;   // Average green field saturation
  bool     calibrated; // Calibration completed flag
};

/**
 * ColorSensor class for reading color sensor data via software I2C
 */
class ColorSensor {
  friend class PeanutKingSoccerV4;   // Only PeanutKingSoccerV4 may construct this module

private:
  I2C_Handle _handles[8];  // I2C handles for 8 color sensors
  uint8_t   _rxBuffer[16] = {}; // Buffer for I2C read operations
  const uint8_t _maxRetry = 5; // Maximum number of retries for I2C read operations

  // Enabled sensors bitmask (bit0=CL1, ..., bit7=CL8)
  uint8_t _enabledMask;

  bool _isWhite[8]; // Track if each sensor detected white color
  GreenBaseline _baseline[8]; // Baseline data of green values for each sensor

  /* Sensor position mapping
   * Maps logical position (Front, Right, Back, Left) to physical sensor ID
   * Default: CL1=Front, CL2=Right, CL3=Back, CL4=Left */
  ColorSensorId _sensorMap[4];

  I2C_Handle &getHandle(ColorSensorId sensorNum);

  // Constructor (accessible only to the friend PeanutKingSoccerV4)
  ColorSensor();

public:
  // init function to initialize the color sensor module
  bool init(void);

  // ============ Sensor Configuration ============

  // Check if a color sensor port is valid
  bool portValidCheck(ColorSensorId sensorNum);

  /**
   * Convert a SensorPos (Front/Right/Back/Left) to the corresponding ColorSensorId (CL1~CL8)
   * based on the current sensor mapping.
   */
  ColorSensorId getPortFromPos(SensorPos pos);

  /**
   * Assign which color sensor (CL1~CL8) is connected to which position
   *
   * `Front` - Sensor ID at front position
   * `Right` - Sensor ID at right position
   * `Back`  - Sensor ID at back position
   * `Left`  - Sensor ID at left position
   *
   * Example: `configuration(CL2, CL1, CL3, CL4)` means
   *   Front position uses sensor CL2,
   *   Right position uses sensor CL1, etc.
   */
  void mapPort(ColorSensorId Front, ColorSensorId Right, ColorSensorId Back, ColorSensorId Left);

  // ============ Sensor Enable/Disable ============

  /**
   * Set which sensors are enabled using bitmask
   * Disabled sensors return 0 when read
   *
   * `mask` - Bitmask (bit0=`CL1`, bit1=`CL2`, ..., bit7=`CL8`)
   *          Example: `0b00001111` (`0x0F`) enables `CL1`-`CL4`
   *
   * Default: `0x0F` (`CL1`-`CL4` sensors enabled)
   */
  void setEnableMask(uint8_t mask);

  /**
   * Enable or disable a single color sensor
   *
   * `sensor`  - Sensor ID (`CL1` - `CL8`)
   * `enabled` - `true` to enable, `false` to disable
   */
  void enable(ColorSensorId sensor, bool enabled);

  /**
   * Check if a sensor is enabled
   *
   * `sensor` - Sensor ID (`CL1` - `CL8`)
   */
  bool isEnabled(ColorSensorId sensor) const;

  // ============ Read Functions ============

  /**
   * Read RGBC raw values from a sensor (register 0x02, 8 bytes)
   * Raw photodiode spectral responsivity data (0-65535 each)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - RGBC structure, returns {0,0,0,0} if disabled
   */
  RGBC readRGBRaw(ColorSensorId sensorNum);

  /**
   * Read HSL values from a sensor (register 0x03, 4 bytes)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - HSL structure, returns {0,0,0} if disabled
   */
  HSL readHSL(ColorSensorId sensorNum);

  /**
   * Read RGB values from a sensor (register 0x08, 3 bytes)
   * Calculated RGB values (0-255 each)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - RGB structure, returns {0,0,0} if disabled
   */
  RGB readRGB(ColorSensorId sensorNum);

  // ============ LED Control Functions ============

  /**
   * Turn on the bottom white LED (register 0x04)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - `true` if successful, `false` if disabled
   */
  bool whiteLedOn(ColorSensorId sensorNum);

  /**
   * Turn off the bottom white LED (register 0x05)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - `true` if successful, `false` if disabled
   */
  bool whiteLedOff(ColorSensorId sensorNum);

  /**
   * Turn on the top RGBW LED (register 0x06)
   * Shows the detected color from address 0x01
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - `true` if successful, `false` if disabled
   */
  bool rgbwLedOn(ColorSensorId sensorNum);

  /**
   * Turn off the top RGBW LED (register 0x07)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - `true` if successful, `false` if disabled
   */
  bool rgbwLedOff(ColorSensorId sensorNum);

  // ============ White Line Detection Functions ============

  /**
   * Calibrate white line baseline (call when sensor is on green field)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   * `samples`   - Number of samples to average (default: 10)
   * 
   * `Returns` - `true` if calibration successful, `false` if disabled or invalid port
   */
  bool calBaseline(ColorSensorId sensorNum, uint8_t samples = 10);

  /**
   * Get the calibrated baseline for a sensor
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - GreenBaseline struct (check `.calibrated` before use)
   */
  GreenBaseline getBaseline(ColorSensorId sensorNum) const;

  /**
   * Check if sensor detects white line (baseline comparison)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - `true` if white line detected
   */
  bool isWhiteLine(ColorSensorId sensorNum);
};

#endif // COLORSENSOR_H