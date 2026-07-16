#include "LedController.h"

LedController::LedController() :
  ledPin{26, 28, 27}
{
}

void LedController::init(void)
{
  // Initialize LED pins as OUTPUT
  for (uint8_t i = 0; i < 3; i++) {
    pinMode(ledPin[i], OUTPUT);
  }
}

void LedController::setOnBrdLED(obBrdLEDCL color)
{
  digitalWrite(ledPin[0], color & 1);
  digitalWrite(ledPin[1], color & 2);
  digitalWrite(ledPin[2], color & 4);
}

void LedController::setOnBrdLED(uint8_t LED, uint8_t status)
{
  if (LED > 2) {
    return; // Invalid LED index
  }
  digitalWrite(ledPin[LED], status);
}