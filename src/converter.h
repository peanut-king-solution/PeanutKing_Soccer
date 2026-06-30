// converter.h
#ifndef CONVERTER_H
#define CONVERTER_H

#include <math.h>

class Converter {
private:
  float _multiplier;
  float _offset;

public:
  // reset the converter to its default state
  Converter();

  /* Configure the converter by using method chaining.
   * e.g. change the compass angle from clockwise to counter-clockwise:
   * compassConverter.config().flip(); */
  Converter& config();
  // Flip the conversion direction (e.g., from clockwise to counter-clockwise)
  Converter& flip();
  // Shift the conversion by a specified amount (e.g., to adjust for an offset)
  Converter& shift(float amount);

  // Normalize an angle to the range [0, 360)
  float normalize(float angle);
  // Apply the conversion to an angle, returning the converted value
  float convert(float angle);
  // Reset the converter to its default state
  void reset();
};

#endif