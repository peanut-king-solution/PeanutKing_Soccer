#ifndef CONVERTER_H
#define CONVERTER_H

#include <math.h>
#include <Arduino.h>

/** Rotation direction for `Converter::rotate()` */
enum RotationDir {
  CW  = 0,  /**< Clockwise rotation (default) */
  CCW = 1   /**< Counter-clockwise rotation */
};

// Utility class for angle conversion and normalization
class Converter {
private:
  float _multiplier;  // Direction multiplier (`-1` for flipped, `1` for normal)
  float _offset;      // Angle offset for calibration

public:
  // Constructor
  Converter();

  // Flip the conversion direction (e.g., from clockwise to counter-clockwise)
  Converter& flip();

  /**
   * Rotates the coordinate system by the specified `angle` (in degrees).
   * `angle` - non-negative rotation angle in degrees [0, 360)
   * `dir`   - rotation direction (`CW` by default)
   *
   * `Returns` - a reference to the current Converter instance for method chaining.
   */
  Converter& rotate(uint16_t angle, RotationDir dir = CW);

  /**
   * Normalize an angle to the range [0, 360)
   * `angle` - the angle in degrees to normalize (can be negative).
   *
   * `Returns` - the normalized angle in degrees [0, 360).
   */
  float normalize(float angle);

  /**
   * Convert an angle using the current multiplier and offset
   * `angle` - the angle in degrees to convert.
   *
   * `Returns` - the converted angle in degrees [0, 360).
   */
  float convert(float angle);

  /**
   * Reset the converter to its default state
   */
  void reset();
};

#endif  // CONVERTER_H