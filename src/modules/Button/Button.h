#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

#include "PeanutKingDef.h"

// Button ID enumeration for identifying buttons
enum ButtonId : uint8_t
{
  Button1 = 0,
  Button2 = 1,
  Button3 = 2,
  Button4 = 3,
};

// Button state enumeration
enum ButtonState : uint8_t
{
  ButtonIdle = 0,
  ButtonPressed = 1,
  ButtonHolding = 2,
  ButtonReleased = 3,
};

/**
 * Button class for reading and managing button states
 * Supports up to 4 buttons with press, hold, release detection
 */
class Button
{
  friend class PeanutKingSoccerV4;   // Only PeanutKingSoccerV4 may construct this module

private:
  const uint8_t  _buttonPin[4];   // Button pins
  ButtonState    _btnStatus[4];   // Buttons' state (index `0-3` = button `1-4`)

  uint16_t _debounceTime = 50;    // Debounce time in milliseconds
  uint16_t _holdTime     = 1000;  // Hold time before `ButtonHolding` triggers (ms)

  // Constructor (accessible only to the friend PeanutKingSoccerV4)
  Button();

public:
  // Initialize button pins as `INPUT_PULLUP` (called by PeanutKingSoccerV4)
  void init(void);

  /**
   * Set the debounce time in milliseconds (default: 50ms)
   * `time` - Debounce time `(ms)`
   */
  void setDebounceTime(uint16_t time);

  /**
   * Set the hold time in milliseconds (default: 1000ms)
   * A button held for this long enters `ButtonHolding`
   * `time` - Hold time `(ms)`
   */
  void setHoldTime(uint16_t time);

  /**
   * Update button state machine
   * Should be called regularly to detect `ButtonPressed`, `ButtonHolding`, `ButtonReleased`
   */
  void update(void);

  /**
   * Read the current state of a button
   * `btn` - Button ID (`Button1` - `Button4`)
   *
   * `Returns` - Current button state, e.g., `ButtonIdle`, `ButtonPressed`, `ButtonHolding`, `ButtonReleased`
   */
  ButtonState readState(ButtonId btn) const;
};

#endif // BUTTON_H