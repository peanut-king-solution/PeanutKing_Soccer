#ifndef LED_H
#define LED_H

#include <Arduino.h>

// On-board RGB LED color enumeration
enum LEDColor : uint8_t {
  LEDOff    = 0,  // 000
  LEDBlue   = 1,  // 001
  LEDGreen  = 2,  // 010
  LEDCyan   = 3,  // 011
  LEDRed    = 4,  // 100
  LEDPurple = 5,  // 101
  LEDYellow = 6,  // 110
  LEDWhite  = 7   // 111
};

/**
 * LED class for controlling on-board RGB LEDs
 * Supports with 8 color modes
 */
class LED
{
  friend class PeanutKingSoccerV4;   // Only PeanutKingSoccerV4 may construct this module

private:
  const uint8_t _ledPin[3];  // On-board LED pins (R, G, B)

  // Constructor (accessible only to the friend PeanutKingSoccerV4)
  LED();

public:
  // Initialize LED pins as OUTPUT (called by PeanutKingSoccerV4)
  void init(void);

  /**
   * Set all on-board LEDs to a specific color
   * `color` - Color to set (`LEDOff` - `LEDWhite`)
   */
  void setLED(LEDColor color);

  /**
   * Set a single on-board LED `on`/`off`
   * `LED`    - RGB Pin index (`0`=Red, `1`=Green, `2`=Blue)
   * `status` - `0` = off, `1` = on
   */
  void setLED(uint8_t LED, uint8_t status);
};

#endif // LED_H