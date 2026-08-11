#include "Button.h"

Button::Button() :
  _buttonPin{22, 23, 24, 25},
  _btnStatus{ButtonIdle, ButtonIdle, ButtonIdle, ButtonIdle}
{
}

void Button::init(void)
{
  // Initialize button pins as INPUT_PULLUP
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(_buttonPin[i], INPUT_PULLUP);
  }
}

void Button::setDebounceTime(uint16_t time)
{
  _debounceTime = time;
}

void Button::setHoldTime(uint16_t time)
{
  _holdTime = time;
}

void Button::update(void)
{
  // time each button was last pressed (for hold detection)
  static uint32_t pressTime[4] = {0};
  // raw pin state (PULLUP idle = HIGH, pressed = LOW)
  static bool     lastRead[4] = {true, true, true, true};
  // time each button was last changed (for debouncing)
  static uint32_t lastChange[4] = {0};

  // Iterate through each button and update its state
  for (uint8_t i = 0; i < 4; i++) {
    // Get the current time
    uint32_t now = millis();
    // Read the current button state (pressed = LOW, idle = HIGH)
    bool pressed = !digitalRead(_buttonPin[i]);

    // Debounce: record raw pin changes and only accept them after `_debounceTime` of stability
    if (pressed != lastRead[i]) {
      lastRead[i] = pressed;
      lastChange[i] = now;
      continue;
    }
    // If the button state has changed but is still within the debounce time, skip processing
    else if (now - lastChange[i] < _debounceTime) {
      continue;  // still bouncing — keep previous state
    }

    switch (_btnStatus[i]) {
    case ButtonIdle:
      // If the button is pressed, transition to ButtonPressed and record the press time
      if (pressed) {
        _btnStatus[i] = ButtonPressed;
        pressTime[i] = now;
      }
      break;

    case ButtonPressed:
      // If the button is still pressed, check if it has been held long enough to transition to ButtonHolding
      if (pressed) {
        if (now - pressTime[i] > _holdTime) _btnStatus[i] = ButtonHolding;
      }
      // If the button is released, transition to ButtonReleased
      else {
        _btnStatus[i] = ButtonReleased;
      }
      break;

    case ButtonHolding:
      // If the button is released, transition to ButtonReleased
      if (!pressed) _btnStatus[i] = ButtonReleased;
      break;

    case ButtonReleased:
      // After the button is released, transition back to ButtonIdle
      _btnStatus[i] = pressed ? ButtonPressed : ButtonIdle;
      if (pressed) pressTime[i] = now;
      break;
    }
  }
}

ButtonState Button::readState(ButtonId btn) const
{
  uint8_t index = static_cast<uint8_t>(btn);
  if (index < 4) { return _btnStatus[index]; }
  return ButtonIdle;
}