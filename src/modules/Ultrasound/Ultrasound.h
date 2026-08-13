#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include <Arduino.h>
#include "pcint/pcint.h"
#include "PeanutKingDef.h"

// Ultrasound port enumeration (U1-U4)
enum UltrasoundId : uint8_t
{
  U1 = 0, // default pos: Front
  U2 = 1, // default pos: Right
  U3 = 2, // default pos: Back
  U4 = 3, // default pos: Left
  UltrasoundMaxCount = 4 // Total number of ultrasound sensors
};

/**
 * Ultrasound class for managing 4 ultrasonic sensors
 * Uses Pin Change Interrupt (PCINT) for non-blocking echo detection
 * Implements round-robin triggering to avoid sensor interference
 */
class Ultrasound
{
  friend class PeanutKingSoccerV4;   // Only PeanutKingSoccerV4 may construct this module

private:
  // Pin configuration
  const uint8_t _trigPin[4]; // Trigger pins
  const uint8_t _echoPin[4]; // Echo pins

  // Sensor mapping: maps logical sensor ID (U1~U4) to physical pin index
  // Default: U1→0 (Front), U2→1 (Right), U3→2 (Back), U4→3 (Left)
  UltrasoundId _ultrasoundMap[4];

  // Enabled sensors bitmask (bit 0=U1, bit 1=U2, bit 2=U3, bit 3=U4)
  volatile uint8_t _enabledMask;

  // Rising edge timestamps for each sensor (millis)
  uint32_t _lastTriggerTime = 0;

  // Round-robin sensor index (0-3)
  volatile uint8_t _currentSeq = 0;
  
  // Rising edge timestamp (micros)
  volatile uint32_t _pulseStart[4] = {0, 0, 0, 0}; 
  
  // Distance results (mm)
  volatile uint16_t _distance[4] = {0, 0, 0, 0};

  // ISR infrastructure
  void handleEcho(uint8_t n);       // Handle echo signal for sensor `n`
  static void echoISR_0();          // ISR for sensor 0
  static void echoISR_1();          // ISR for sensor 1
  static void echoISR_2();          // ISR for sensor 2
  static void echoISR_3();          // ISR for sensor 3
  static void (*_echoISR_ptr[4])(); // Array of ISR function pointers for each sensor

  // Advance round-robin to next enabled sensor and send trigger pulse
  void _triggerNext(void);

  // Constructor (accessible only to the friend PeanutKingSoccerV4)
  Ultrasound();

public:
  // Initialize ultrasonic sensor pins and attach PCINT interrupts
  void init(void);

  /**
   * Set the sensor mapping: which Ultrasound port is at each physical position.
   * Default mapping is identity (U1=Front, U2=Right, U3=Back, U4=Left).
   *
   * `front` - Ultrasound `port` plugged at the front position
   * `right` - Ultrasound `port` plugged at the right position
   * `back`  - Ultrasound `port` plugged at the back position
   * `left`  - Ultrasound `port` plugged at the left position
   */
  void mapPort(UltrasoundId Front, UltrasoundId Right, UltrasoundId Back, UltrasoundId Left);
  // Check if a sensor port is valid (U1~U4)
  bool portValidCheck(UltrasoundId port);
  /**
   * Convert a sensor position (Front/Right/Back/Left) to the corresponding UltrasoundId (U1~U4)
   * based on the current sensor mapping.
   */
  UltrasoundId getPortFromPos(SensorPos pos);

  /**
   * Set the enabled sensors mask.
   *
   * `mask` - Bitmask where each bit represents a sensor (bit 0=U1, bit 1=U2, bit 2=U3, bit 3=U4)
   */
  void setEnableMask(uint8_t mask);
  /**
   * Enable or disable a single sensor
   *
   * `port`  - Sensor port (`U1`, `U2`, `U3`, `U4`)
   * `enabled` - `true` to enable, `false` to disable
   */
  void enable(UltrasoundId port, bool enabled);
  /**
   * Check if a single sensor is enabled.
   *
   * `port` - Sensor port (`U1`, `U2`, `U3`, `U4`)
   *
   * `Returns` - `true` if the sensor is enabled, `false` otherwise
   */
  bool isEnabled(UltrasoundId port) const;

  /**
   * Read distance from a specified port (U1~U4)
   * Triggers sensors in round-robin fashion (one sensor per call)
   *
   * `port` - Sensor port (`U1`, `U2`, `U3`, `U4`)
   *
   * `Returns` - Distance in mm (0~4500mm)
   */
  uint16_t read(UltrasoundId port);
};

#endif // ULTRASOUND_H
