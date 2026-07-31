#ifndef LED_H
#define LED_H

#include <Arduino.h>

// On-board RGB LED color enumeration
enum class LEDColor : uint8_t {
  OFF,    // 000
  BLUE,   // 001
  GREEN,  // 010
  CYAN,   // 011
  RED,    // 100
  PURPLE, // 101
  YELLOW, // 110
  WHITE   // 111
};

/**
 * LED class for controlling on-board RGB LEDs
 * Supports with 8 color modes
 */
class LED
{
private:
  const uint8_t _ledPin[3];  // On-board LED pins (R, G, B)

public:
  // Constructor
  LED();

  // Initialize LED pins as OUTPUT
  void init(void);

  /**
   * Set all on-board LEDs to a specific color
   * `color` - Color to set (`LEDColor::OFF` - `LEDColor::WHITE`)
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