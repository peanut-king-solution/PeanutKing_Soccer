#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#include <pcint.h>

// Ultrasonic Sensor ID enumeration
typedef enum
{
  U1 = 0, // Front
  U2 = 1, // Right
  U3 = 2, // Back
  U4 = 3  // Left
} ULTR_SENSOR;

/**
 * Ultrasonic class for managing 4 ultrasonic sensors
 * Uses Pin Change Interrupt (PCINT) for non-blocking echo detection
 * Implements round-robin triggering to avoid sensor interference
 */
class Ultrasonic
{
private:
  // Pin configuration
  const uint8_t trigPin[4]; // Trigger pins
  const uint8_t echoPin[4]; // Echo pins

  // Sensor mapping: maps logical sensor ID (U1~U4) to physical pin index
  // Default: U1→0 (Front), U2→1 (Right), U3→2 (Back), U4→3 (Left)
  uint8_t xsoundMap[4];

  // Timing data
  uint32_t pulseStart[4];   // Rising edge timestamp (micros)
  uint32_t lastTriggerTime; // Rate limiting timestamp (millis)
  uint8_t  currentSeq;      // Round-robin sensor index (0-3)

  // Distance results (mm)
  uint16_t distance[4];

  // ISR infrastructure (static pointer pattern)
  static Ultrasonic *_instance;    // Pointer to the single instance of Ultrasonic for ISR access
  void handleEcho(uint8_t n);      // Handle echo signal for sensor `n`
  static void echoISR_0();         // ISR for sensor 0
  static void echoISR_1();         // ISR for sensor 1
  static void echoISR_2();         // ISR for sensor 2
  static void echoISR_3();         // ISR for sensor 3
  static void (*echoISR_ptr[4])(); // Array of ISR function pointers for each sensor

public:
  // Constructor
  Ultrasonic();

  // Initialize ultrasonic sensor pins and attach PCINT interrupts
  void init(void);

  /**
   * Assign which xsound port (U1~U4) is connected to which position (Front, Right, Back, Left)
   *
   * `Front` - Sensor ID plugged at the front position
   * `Right` - Sensor ID plugged at the right position
   * `Back`  - Sensor ID plugged at the back position
   * `Left`  - Sensor ID plugged at the left position
   *
   * Example: `mapXsound(U2, U1, U4, U3)` means
   *   `Front pin` is actually sensor `U2`,
   *   `Right pin` is actually sensor `U1`, etc.
   */
  void mapXsounds(ULTR_SENSOR Front, ULTR_SENSOR Right, ULTR_SENSOR Back, ULTR_SENSOR Left);

  /**
   * Read distance from a specified sensor
   * Triggers sensors in round-robin fashion (one sensor per call)
   *
   * `sensor` - Sensor ID (`U1`, `U2`, `U3`, `U4`)
   *
   * `Returns` - Distance in mm (0~4500mm)
   */
  uint16_t read(ULTR_SENSOR sensor);
};

#endif // ULTRASONIC_H
