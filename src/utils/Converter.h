#ifndef CONVERTER_H
#define CONVERTER_H

#include <math.h>


// Utility class for angle conversion and normalization
class Converter {
private:
  float _multiplier;  // Direction multiplier (`-1` for flipped, `1` for normal)
  float _offset;      // Angle offset for calibration

public:
  // Constructor
  Converter();

  /* Configure the converter by using method chaining.
   * 
   * e.g. change the compass angle from clockwise to counter-clockwise:
   * `robot.compass.converter.config().flip();`
   */
  Converter& config();

  // Flip the conversion direction (e.g., from clockwise to counter-clockwise)
  Converter& flip();
  
  // Shift the angle by a specified amount (in degrees)
  Converter& shift(float amount);

  // Normalize an angle to the range [0, 360)
  float normalize(float angle);

  // Convert an angle using the current multiplier and offset
  float convert(float angle);

  // Reset the converter to its default state
  void reset();
};

#endif  // CONVERTER_H