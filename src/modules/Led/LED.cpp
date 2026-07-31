#include "LED.h"

LED::LED() :
  _ledPin{26, 28, 27}
{
}

void LED::init(void)
{
  // Initialize LED pins as OUTPUT
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(_ledPin[i], OUTPUT);
  }
}

void LED::setLED(LEDColor color)
{
  uint8_t val = static_cast<uint8_t>(color);
  digitalWrite(_ledPin[0], val & 1);
  digitalWrite(_ledPin[1], val & 2);
  digitalWrite(_ledPin[2], val & 4);
}

void LED::setLED(uint8_t LED, uint8_t status)
{
  if (LED > 2) {
    return; // Invalid LED index
  }
  digitalWrite(_ledPin[LED], status);
}