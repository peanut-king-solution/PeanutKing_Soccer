#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

#include "PeanutKingDef.h"

// Button ID enumeration for identifying buttons
typedef enum : uint8_t
{
  BTN_1 = 1,
  BTN_2 = 2,
  BTN_3 = 3,
  BTN_4 = 4,
} ButtonId;

/**
 * Button class for reading and managing button states
 * Supports up to 4 buttons with TAP, PRESS, HOLD, TAP2, TAP3 detection
 */
class Button
{
private:
  const uint8_t  buttonPin[4];   // Button pins
  ButtonStatus   btnStatus[4];   // Buttons' status (index `0-3` = button `1-4`)

public:
  // Constructor
  Button();

  // Initialize button pins as `INPUT_PULLUP`
  void init(void);

  /**
   * Read button state (pressed or not)
   * `btn` - Button ID (`BTN_1` - `BTN_4`)
   *
   * `Returns` - `true` if pressed, `false` otherwise
   */
  bool read(ButtonId btn);

  /**
   * Update button state machine
   * Should be called regularly to detect `TAP`, `PRESS`, `HOLD`, etc.
   */
  void update(void);
  /**
   * Get the current status of a button
   * `btn` - Button ID (`BTN_1` - `BTN_4`)
   *
   * `Returns` - Current button status, e.g., `TAP`, `PRESS`, `HOLD`, etc.
   */
  ButtonStatus getStatus(ButtonId btn);
};

#endif // BUTTON_H