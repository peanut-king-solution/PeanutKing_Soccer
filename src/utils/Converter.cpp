#include "Converter.h"

float Converter::normalize(float angle) {
  float r = fmodf(angle, 360.0f);
  if (r < 0.0f) r += 360.0f;
  return r;
}

Converter::Converter() {
  reset();
}

Converter& Converter::flip() {
  _multiplier = -_multiplier;
  _offset = -_offset;
  return *this;
}

Converter& Converter::rotate(uint16_t angle, RotationDir dir) {
  // `_offset` is stored additively as a shift: a positive offset shifts the
  // coordinate counter-clockwise (CCW). So:
  //   rotate(90, CW)  -> offset -= 90  (coordinate rotates clockwise)
  //   rotate(90, CCW) -> offset += 90
  // The direction is decided purely by `dir`; `angle` is always non-negative.

  _offset += (dir == CW ? -(int16_t)angle : (int16_t)angle);
  return *this;
}

float Converter::convert(float angle) {
  return normalize(_multiplier * angle + _offset);
}

void Converter::reset() {
  _multiplier = 1.0f;
  _offset = 0.0f;
}