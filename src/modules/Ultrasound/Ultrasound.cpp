#include "Ultrasound.h"

/* =============================================================================
 *                              Static Members
 * ============================================================================= */

// Pointer to the single instance of Ultrasound for ISR access
Ultrasound *Ultrasound::_instance = nullptr;

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
  _ultrasoundMap{0, 1, 2, 3},   // default mapping (no mapping applied)
  _pulseStart{0, 0, 0, 0},      // Rising edge timestamps for each sensor
  _lastTriggerTime(0),          // Rate limiting timestamp (millis)
  _currentSeq(0),               // Round-robin sensor index (0-3)
  _distance{0, 0, 0, 0},        // Distance results (mm)
  _enabledMask(0x0F)            // all enabled by default
{
  // Set static instance pointer for ISR callbacks
  _instance = this;
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

void Ultrasound::setMap(UltrasoundId front, UltrasoundId right, UltrasoundId back, UltrasoundId left)
{
  // Map logical sensor IDs to physical positions
  _ultrasoundMap[0] = (uint8_t)front;
  _ultrasoundMap[1] = (uint8_t)right;
  _ultrasoundMap[2] = (uint8_t)back;
  _ultrasoundMap[3] = (uint8_t)left;
}

UltrasoundId Ultrasound::getPortFromPos(Position pos)
{
  // Convert Position enum to index (0-3)
  uint8_t idx = (uint8_t)pos;
  if (idx >= 4)  return U1;
  // Return the corresponding UltrasoundId based on the current mapping
  return (UltrasoundId)_ultrasoundMap[idx];
}

/* =============================================================================
 *                              Single Sensor Enable / Disable
 * ============================================================================= */

void Ultrasound::enable(UltrasoundId port, bool enabled)
{
  // Validate port index
  uint8_t n = (uint8_t)port;
  if (n >= 4) return;

  // Update the enabled mask for the specified sensor
  if (enabled) {
    _enabledMask |= (1 << n);
  } 
  else {
    _enabledMask &= ~(1 << n);
  }
}

void Ultrasound::setEnabledByPos(bool front, bool right, bool back, bool left)
{
  enable(getPortFromPos(Position::FRONT), front);
  enable(getPortFromPos(Position::RIGHT), right);
  enable(getPortFromPos(Position::BACK), back);
  enable(getPortFromPos(Position::LEFT), left);
}

void Ultrasound::enableAll(bool enabled)
{
  _enabledMask = enabled ? 0x0F : 0x00;
}

/* =============================================================================
 *                              Distance Reading
 * ============================================================================= */

uint16_t Ultrasound::read(UltrasoundId port)
{
  // Validate port index
  uint8_t n = (uint8_t)port;
  if (n >= 4) { return 0; }

  // Check if the requested sensor is enabled
  if (!(_enabledMask & (1 << n))) { return 0; }

  // Rate limiting - minimum 30ms between trigger pulses
  if (millis() - _lastTriggerTime < 30) {
    return _distance[n];
  }
  
  // Round-robin trigger via private helper
  _triggerNext();

  return _distance[n];
}

/* =============================================================================
 *                              Round-Robin Trigger
 * ============================================================================= */

void Ultrasound::_triggerNext(void)
{
  // Find next enabled sensor in round-robin order
  for (uint8_t i = 0; i < 4; i++) {
    _currentSeq = (_currentSeq >= 3) ? 0 : _currentSeq + 1;
    if (_enabledMask & (1 << _currentSeq))
    {
      break; // Found an enabled sensor
    }
  }

  // Only trigger if the selected sensor is enabled
  if (_enabledMask & (1 << _currentSeq)) {
    // Send 10us trigger pulse on the physical pin
    digitalWrite(_trigPin[_currentSeq], LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin[_currentSeq], HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin[_currentSeq], LOW);

    // Update last trigger time
    _lastTriggerTime = millis();
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
  if (!(_enabledMask & (1 << n))) { return; }

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

void Ultrasound::echoISR_0() { _instance->handleEcho(0); }
void Ultrasound::echoISR_1() { _instance->handleEcho(1); }
void Ultrasound::echoISR_2() { _instance->handleEcho(2); }
void Ultrasound::echoISR_3() { _instance->handleEcho(3); }
