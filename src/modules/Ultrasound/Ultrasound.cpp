#include "Ultrasound.h"

/* =============================================================================
 *                              File-scope Static
 * ============================================================================= */

// Pointer to the sole instance of Ultrasound, used by ISR callbacks.
// Declared as file-scope static so it is hidden from other translation units.
static Ultrasound *s_instance = nullptr;

void (*Ultrasound::_echoISR_ptr[4])() = {
  Ultrasound::echoISR_0,
  Ultrasound::echoISR_1,
  Ultrasound::echoISR_2,
  Ultrasound::echoISR_3
};

/* =============================================================================
 *                              Constructor
 * ============================================================================= */

Ultrasound::Ultrasound() :
  _trigPin{49, 48, 47, 46},     // U1, U2, U3, U4
  _echoPin{A15, A14, A13, A12}, // U1, U2, U3, U4
  _ultrasoundMap{U1, U2, U3, U4},   // default mapping (no mapping applied)
  _enabledMask(0x0F)            // all enabled by default
{
  // Ensure only one instance of Ultrasound exists
  if (s_instance == nullptr) {
    s_instance = this;
  }
}

/* =============================================================================
 *                              Initialization
 * ============================================================================= */

void Ultrasound::init(void)
{
  // Initialize ultrasonic pins and attach PCINT interrupts
  for (uint8_t i = 0; i < 4; i++)
  {
    pinMode(_trigPin[i], OUTPUT);
    pinMode(_echoPin[i], INPUT);
    PcInt::attachInterrupt(_echoPin[i], _echoISR_ptr[i], CHANGE);
  }
  // Small delay to ensure pins are stable before starting measurements
  delay(10);
}

/* =============================================================================
 *                              Sensor Mapping
 * ============================================================================= */

void Ultrasound::mapPort(UltrasoundId Front, UltrasoundId Right, UltrasoundId Back, UltrasoundId Left)
{
  // Map logical sensor IDs to physical positions
  _ultrasoundMap[0] = (uint8_t)Front;
  _ultrasoundMap[1] = (uint8_t)Right;
  _ultrasoundMap[2] = (uint8_t)Back;
  _ultrasoundMap[3] = (uint8_t)Left;
}

bool Ultrasound::portValidCheck(UltrasoundId port)
{
  return (port >= U1 && port <= U4);
}

UltrasoundId Ultrasound::getPortFromPos(SensorPos pos)
{
  // Validate position input
  if (pos < Front || pos > Left) { return UltrasoundMaxCount; }
  // Return the corresponding UltrasoundId based on the current mapping
  return _ultrasoundMap[pos];
}

/* =============================================================================
 *                              Single Sensor Enable / Disable
 * ============================================================================= */

void Ultrasound::setEnableMask(uint8_t mask)
{
  if (mask > 0x0F) { mask = 0x0F; } // Ensure only lower 4 bits are used
  _enabledMask = mask;  // Set the enabled sensors bitmask (bit 0=U1, bit 1=U2, bit 2=U3, bit 3=U4)
}

void Ultrasound::enable(UltrasoundId port, bool enabled)
{
  // Validate port index
  if (!portValidCheck(port)) { return; }

  // Update the enabled mask for the specified sensor
  if (enabled) {
    _enabledMask |= (1 << port);
  } 
  else {
    _enabledMask &= ~(1 << port);
  }
}

bool Ultrasound::isEnabled(UltrasoundId port) const
{
  // Validate port index
  if (!portValidCheck(port)) { return false; }

  // Check if the specified sensor is enabled in the mask
  return (_enabledMask & (1 << port)) != 0;
}

/* =============================================================================
 *                              Distance Reading
 * ============================================================================= */

uint16_t Ultrasound::read(UltrasoundId port)
{
  // Return 65535 if the port is invalid or disabled
  if (!portValidCheck(port) || !isEnabled(port)) { return 65535; }

  // Rate limiting - minimum 30ms between trigger pulses
  if (millis() - _lastTriggerTime < 30) {
    return _distance[port];
  }
  
  // Round-robin trigger via private helper
  _triggerNext();

  return _distance[port];
}

/* =============================================================================
 *                              Round-Robin Trigger
 * ============================================================================= */

void Ultrasound::_triggerNext(void)
{
  // Find next enabled sensor in round-robin order
  for (uint8_t i = 0; i < 4; i++) {
    _currentSeq = (_currentSeq >= 3) ? 0 : _currentSeq + 1;
    if (isEnabled(_currentSeq))
    {
      // Send 10us trigger pulse on the physical pin
      digitalWrite(_trigPin[_currentSeq], LOW);
      delayMicroseconds(2);
      digitalWrite(_trigPin[_currentSeq], HIGH);
      delayMicroseconds(10);
      digitalWrite(_trigPin[_currentSeq], LOW);

      // Update last trigger time
      _lastTriggerTime = millis();
      return; // Found an enabled sensor
    }
  }
}

/* =============================================================================
 *                              ISR Handler
 * ============================================================================= */

void Ultrasound::handleEcho(uint8_t n)
{
  // Only process if this is the currently active sensor
  if (_currentSeq != n) { return; }

  // Skip if this sensor is disabled
  if (!isEnabled(n)) { return; }

  if (digitalRead(_echoPin[n])) {
    // Rising edge - capture start time
    _pulseStart[n] = micros();
  }
  else {
    // Falling edge - calculate pulse duration
    uint32_t duration = micros() - _pulseStart[n];

    // Convert pulse duration to distance (mm)
    // Sound speed: 340 m/s → 0.34 mm/us, round trip → 0.17 mm/us
    float dist = (float)duration * 0.17f;

    // Update distance if within valid range (max 4500mm)
    if (dist < 4500)
    {
      _distance[n] = (uint16_t)round(dist);
    }
  }
}

/* =============================================================================
 *                              Static ISR Wrappers
 * ============================================================================= */

void Ultrasound::echoISR_0() { s_instance->handleEcho(0); }
void Ultrasound::echoISR_1() { s_instance->handleEcho(1); }
void Ultrasound::echoISR_2() { s_instance->handleEcho(2); }
void Ultrasound::echoISR_3() { s_instance->handleEcho(3); }
