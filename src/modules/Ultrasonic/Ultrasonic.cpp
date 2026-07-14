#include "Ultrasonic.h"

/* =============================================================================
 *                              Static Members
 * ============================================================================= */

// Pointer to the single instance of Ultrasonic for ISR access
Ultrasonic *Ultrasonic::_instance = nullptr;

void (*Ultrasonic::echoISR_ptr[4])() = {
  Ultrasonic::echoISR_0,
  Ultrasonic::echoISR_1,
  Ultrasonic::echoISR_2,
  Ultrasonic::echoISR_3
};

/* =============================================================================
 *                              Constructor
 * ============================================================================= */

Ultrasonic::Ultrasonic() :
  trigPin{49, 48, 47, 46},     // U1, U2, U3, U4
  echoPin{A15, A14, A13, A12}, // U1, U2, U3, U4
  xsoundMap{0, 1, 2, 3},       // default mapping (no mapping applied)
  pulseStart{0, 0, 0, 0},
  lastTriggerTime(0),
  currentSeq(0),
  distance{0, 0, 0, 0},
  enabledMask(0x0F)            // all enabled by default
{
  // Set static instance pointer for ISR callbacks
  _instance = this;
}

/* =============================================================================
 *                              Initialization
 * ============================================================================= */

void Ultrasonic::init(void)
{
  // Initialize ultrasonic pins and attach PCINT interrupts
  for (uint8_t i = 0; i < 4; i++)
  {
    pinMode(trigPin[i], OUTPUT);
    pinMode(echoPin[i], INPUT);
    PcInt::attachInterrupt(echoPin[i], echoISR_ptr[i], CHANGE);
  }
  // Small delay to ensure pins are stable before starting measurements
  delay(10);
}

/* =============================================================================
 *                              Sensor Mapping
 * ============================================================================= */

void Ultrasonic::mapXsounds(ULTR_SENSOR Front, ULTR_SENSOR Right, ULTR_SENSOR Back, ULTR_SENSOR Left)
{
  // Map logical sensor IDs to physical positions
  xsoundMap[0] = (uint8_t)Front;
  xsoundMap[1] = (uint8_t)Right;
  xsoundMap[2] = (uint8_t)Back;
  xsoundMap[3] = (uint8_t)Left;
}

/* =============================================================================
 *                              Enable / Disable
 * ============================================================================= */

void Ultrasonic::setEnabled(bool u1, bool u2, bool u3, bool u4)
{
  // Convert 4 boolean parameters to bitmask
  enabledMask = 0;
  if (u1) enabledMask |= (1 << 0);
  if (u2) enabledMask |= (1 << 1);
  if (u3) enabledMask |= (1 << 2);
  if (u4) enabledMask |= (1 << 3);
}

/* =============================================================================
 *                              Single Sensor Enable / Disable
 * ============================================================================= */

void Ultrasonic::enableSensor(ULTR_SENSOR sensor, bool enabled)
{
  uint8_t n = (uint8_t)sensor;
  if (n >= 4) return;

  if (enabled) {
    enabledMask |= (1 << n);
  } else {
    enabledMask &= ~(1 << n);
  }
}

/* =============================================================================
 *                              All Sensors Enable / Disable
 * ============================================================================= */

void Ultrasonic::enableAll(bool enabled)
{
  enabledMask = enabled ? 0x0F : 0x00;
}

/* =============================================================================
 *                              Distance Reading
 * ============================================================================= */

uint16_t Ultrasonic::read(ULTR_SENSOR sensor)
{
  // Validate sensor index
  uint8_t n = (uint8_t)sensor;
  if (n >= 4) return 0;

  // Check if the requested sensor is enabled
  if (!(enabledMask & (1 << n))) return 0;

  // Map logical sensor to physical index
  uint8_t physIndex = xsoundMap[n];

  // Rate limiting - minimum 30ms between trigger pulses
  if (millis() - lastTriggerTime < 30) return distance[physIndex];

  // Round-robin: find next enabled sensor
  // This ensures only enabled sensors are triggered in rotation
  for (uint8_t i = 0; i < 4; i++) {
    currentSeq = (currentSeq >= 3) ? 0 : currentSeq + 1;
    if (enabledMask & (1 << currentSeq)) {
      break;  // Found an enabled sensor
    }
  }

  // Only trigger if the selected sensor is enabled
  if (enabledMask & (1 << currentSeq)) {
    // Send 10us trigger pulse on the physical pin
    digitalWrite(trigPin[currentSeq], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin[currentSeq], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin[currentSeq], LOW);

    // Update last trigger time
    lastTriggerTime = millis();
  }

  return distance[physIndex];
}

/* =============================================================================
 *                              ISR Handler
 * ============================================================================= */

void Ultrasonic::handleEcho(uint8_t n)
{
  // Only process if this is the currently active sensor
  if (currentSeq != n) return;

  // Skip if this sensor is disabled
  if (!(enabledMask & (1 << n))) return;

  if (digitalRead(echoPin[n])) {
    // Rising edge - capture start time
    pulseStart[n] = micros();
  } else {
    // Falling edge - calculate pulse duration
    pulseStart[n] = micros() - pulseStart[n];
  }

  // Convert pulse duration to distance (mm)
  // Sound speed: 340 m/s → 0.34 mm/us, round trip → 0.17 mm/us
  float dist = (float)pulseStart[n] * 0.17f;

  // Update distance if within valid range (max 4500mm)
  if (dist < 4500) {
    distance[n] = (uint16_t)round(dist);
  }
}

/* =============================================================================
 *                              Static ISR Wrappers
 * ============================================================================= */

void Ultrasonic::echoISR_0() { _instance->handleEcho(0); }
void Ultrasonic::echoISR_1() { _instance->handleEcho(1); }
void Ultrasonic::echoISR_2() { _instance->handleEcho(2); }
void Ultrasonic::echoISR_3() { _instance->handleEcho(3); }
