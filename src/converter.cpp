#include "converter.h"

// Normalize an angle to the range [0, 360)
float Converter::normalize(float angle) {
  float r = fmodf(angle, 360.0f);
  if (r < 0.0f) r += 360.0f;
  return r;
}

// reset the converter to its default state
Converter::Converter() {
  reset();
}

/*
Configure the converter by using method chaining.
e.g. change the compass angle from clockwise to counter-clockwise:
compassConverter.config().flip();
*/
Converter& Converter::config() {
  reset();
  return *this;
}

// Flip the conversion direction (e.g., from clockwise to counter-clockwise)
Converter& Converter::flip() {
  _multiplier = -_multiplier;
  _offset = -_offset;
  return *this;
}

// Shift the conversion by a specified amount (e.g., to adjust for an offset)
Converter& Converter::shift(float amount) {
  _offset += amount;
  return *this;
}

// Apply the conversion to an angle, returning the converted value
float Converter::convert(float angle) {
  return normalize(_multiplier * angle + _offset);
}

// Reset the converter to its default state
void Converter::reset() {
  _multiplier = 1.0f;
  _offset = 0.0f;
}