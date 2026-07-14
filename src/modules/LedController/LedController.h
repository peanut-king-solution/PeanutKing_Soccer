#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>

// On-board RGB LED color enumeration
typedef enum : uint8_t {
  LED_OFF,    // 000
  LED_BLUE,   // 001
  LED_GREEN,  // 010
  LED_CYAN,   // 011
  LED_RED,    // 100
  LED_PURPLE, // 101
  LED_YELLOW, // 110
  LED_WHITE   // 111
} obBrdLEDCL;

/**
 * LedController class for controlling on-board RGB LEDs
 * Supports with 8 color modes
 */
class LedController
{
private:
  const uint8_t ledPin[3];   // On-board LED pins (R, G, B)

public:
  // Constructor
  LedController();

  // Initialize LED pins as OUTPUT
  void init(void);

  /**
   * Set all on-board LEDs to a specific color
   * `color` - Color to set (`LED_OFF` - `LED_WHITE`)
   */
  void setOnBrdLED(obBrdLEDCL color);

  /**
   * Set a single on-board LED `on`/`off`
   * `LED`    - LED index (`0-2`)
   * `status` - `0` = off, `1` = on
   */
  void setOnBrdLED(uint8_t LED, uint8_t status);
};

#endif // LED_CONTROLLER_H