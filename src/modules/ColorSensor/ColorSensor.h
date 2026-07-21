#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#define COLOR_SENSOR_ADDRESS 0x11 // Color sensor I2C address

#include <Arduino.h>
#include "PeanutKingDef.h"
#include "modules/I2C/i2cManager.h"

/**
 * Color sensor indices (CL1-CL8)
 */
typedef enum : uint8_t {
  CL1 = 0, CL2, CL3, CL4, CL5, CL6, CL7, CL8
} CLR_SENSOR_ID;

typedef enum : uint8_t {
  CLR_BLACK = 0, CLR_WHITE, CLR_GREY, CLR_RED, CLR_GREEN, CLR_BLUE, CLR_YELLOW, CLR_CYAN
} COLOR_IDX;

/**
 * RGBC raw data structure (register 0x02)
 * Raw photodiode spectral responsivity data (0-65535 each)
 */
typedef struct {
  uint16_t r;    // Red raw
  uint16_t g;    // Green raw
  uint16_t b;    // Blue raw
  uint16_t c;    // Clear raw
} rgbc_t;

/**
 * ColorSensor class for reading color sensor data via software I2C
 */
class ColorSensor {
private:
  I2C_Handle _handles[8];  // I2C handles for 8 color sensors
  uint8_t   _rxBuffer[16]; // Buffer for I2C read operations
  uint8_t   _enabledMask;  // Enabled sensors bitmask (bit0=CL1, ..., bit7=CL8)

  bool _isWhite[8]; // Track if each sensor detected white color

  // Calibrated baseline of green values for white line detection
  struct WhiteLineBaseline {
    uint16_t greenHue;   // Average green field hue
    uint8_t  greenLight; // Average green field lightness
    uint8_t  greenSat;   // Average green field saturation
    bool     done;       // Calibration completed flag
  } _baseline[8];        // Baseline data for each sensor

  /* Sensor position mapping
   * Maps logical position (Front, Right, Back, Left) to physical sensor ID
   * Default: CL1=Front, CL2=Right, CL3=Back, CL4=Left */
  CLR_SENSOR_ID _sensorMap[4];

  I2C_Handle &getHandle(CLR_SENSOR_ID sensorNum);

public:
  // Constructor
  ColorSensor();

  // init function to initialize the color sensor module
  bool init(void);

  // ============ Sensor Configuration ============

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
  void configuration(CLR_SENSOR_ID Front, CLR_SENSOR_ID Right, CLR_SENSOR_ID Back, CLR_SENSOR_ID Left);

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
  void setEnabled(uint8_t mask);

  /**
   * Enable or disable a single sensor
   *
   * `sensor`  - Sensor ID (`CL1` - `CL8`)
   * `enabled` - `true` to enable, `false` to disable
   */
  void enableSensor(CLR_SENSOR_ID sensor, bool enabled);

  /**
   * Check if a sensor is enabled
   *
   * `sensor` - Sensor ID (`CL1` - `CL8`)
   */
  bool isEnabled(CLR_SENSOR_ID sensor) const;

  // ============ Read Functions ============

  /**
   * Read color index from a sensor (register 0x01)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - Color index (`0`=Black ... `7`=Cyan), returns `0` if disabled
   */
  uint8_t readColor(CLR_SENSOR_ID sensorNum);

  /**
   * Read RGBC raw values from a sensor (register 0x02, 8 bytes)
   * Raw photodiode spectral responsivity data (0-65535 each)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - RGBC structure, returns {0,0,0,0} if disabled
   */
  rgbc_t readRGBRaw(CLR_SENSOR_ID sensorNum);

  /**
   * Read HSL values from a sensor (register 0x03, 4 bytes)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - HSL structure, returns {0,0,0} if disabled
   */
  hsl_t readHSL(CLR_SENSOR_ID sensorNum);

  /**
   * Read RGB values from a sensor (register 0x08, 3 bytes)
   * Calculated RGB values (0-255 each)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - RGB structure, returns {0,0,0} if disabled
   */
  rgb_t readRGB(CLR_SENSOR_ID sensorNum);

  // ============ LED Control Functions ============

  /**
   * Turn on the bottom white LED (register 0x04)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - `true` if successful, `false` if disabled
   */
  bool whiteLedOn(CLR_SENSOR_ID sensorNum);

  /**
   * Turn off the bottom white LED (register 0x05)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - `true` if successful, `false` if disabled
   */
  bool whiteLedOff(CLR_SENSOR_ID sensorNum);

  /**
   * Turn on the top RGBW LED (register 0x06)
   * Shows the detected color from address 0x01
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - `true` if successful, `false` if disabled
   */
  bool rgbwLedOn(CLR_SENSOR_ID sensorNum);

  /**
   * Turn off the top RGBW LED (register 0x07)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - `true` if successful, `false` if disabled
   */
  bool rgbwLedOff(CLR_SENSOR_ID sensorNum);

  // ============ White Line Detection Functions ============

  /**
   * Calibrate white line baseline (call when sensor is on green field)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   * `samples`   - Number of samples to average (default: 10)
   */
  void calBaseline(CLR_SENSOR_ID sensorNum, uint8_t samples = 10);

  /**
   * Check if calibration is done for a sensor
   */
  bool isCalibrated(CLR_SENSOR_ID sensorNum) const;

  /**
   * Check if sensor detects white line (baseline comparison)
   * `sensorNum` - Sensor ID (`CL1` - `CL8`)
   *
   * `Returns` - `true` if white line detected
   */
  bool isWhiteLine(CLR_SENSOR_ID sensorNum);
};

#endif // COLORSENSOR_H