#include "Converter.h"

float Converter::normalize(float angle) {
  float r = fmodf(angle, 360.0f);
  if (r < 0.0f) r += 360.0f;
  return r;
}

Converter::Converter() {
  reset();
}

Converter& Converter::config() {
  reset();
  return *this;
}

Converter& Converter::flip() {
  _multiplier = -_multiplier;
  _offset = -_offset;
  return *this;
}

Converter& Converter::shift(float amount) {
  _offset += amount;
  return *this;
}

float Converter::convert(float angle) {
  return normalize(_multiplier * angle + _offset);
}

void Converter::reset() {
  _multiplier = 1.0f;
  _offset = 0.0f;
}