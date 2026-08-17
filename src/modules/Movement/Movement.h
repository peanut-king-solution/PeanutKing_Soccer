#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "PeanutKingDef.h"
#include "utils/Converter.h"
#include "utils/PIDController.h"

/**
 * Wheel speeds computed for each physical position (position-based).
 * Mapping to actual motor ports is handled by the caller (V4 wrapper).
 */
struct WheelSpeeds {
  int16_t leftFront;   // LeftFront
  int16_t rightFront;  // RightFront
  int16_t rightBack;   // RightBack
  int16_t leftBack;    // LeftBack
};

class Movement
{
  friend class PeanutKingSoccerV4;   // Only PeanutKingSoccerV4 may construct this module

  /**
   * Constructor (accessible only to the friend PeanutKingSoccerV4)
   */
  Movement();

public:

  // --- Tuning parameters (public) ---

  /** Compass dead zone for rotation correction. Small errors below this are ignored. */
  float compassDeadZone = 0.05f;

  /** Minimum rotation speed. When PID correction is smaller than this, clamp to it. */
  float minRotateSpeed = 60.0f;

  // --- Motion computation (returns WheelSpeeds, does NOT drive motors) ---

  /**
   * Compute wheel speeds at angle with speed and rotation
   * `mAngle`  - Movement angle `(0-360°)`
   * `mSpeed`  - Movement speed `(0-255)`
   * `rotate`  - Rotation speed `(-255 to +255)`, positive=`CW`, negative=`CCW`
   * 
   * `Returns` - Wheel speeds for each wheel (position-based)
   */
  WheelSpeeds byAngle(float mAngle, float mSpeed, float rotate);

  /**
   * Compute wheel speeds with compass correction
   * `mAngle`          - Movement angle `(0-360°)`
   * `mSpeed`          - Movement speed `(0-255)`
   * `compassReading`  - Current compass heading `(0-360°)`
   * 
   * `Returns` - Wheel speeds for each wheel (position-based)
   */
  WheelSpeeds withCorr(float mAngle, float mSpeed, float compassReading);

  /**
   * Compute wheel speeds to prevent moving out of bounds
   * `mAngle`     - Movement angle `(0-360°)`
   * `mSpeed`     - Movement speed `(0-255)`
   * `isOutBound` - Array indicating if each direction is out of bounds (Front, Right, Back, Left)
   *
   * `Returns` - Wheel speeds that prevent out-of-bounds movement
   */
  WheelSpeeds outBoundPrevent(float mAngle, float mSpeed, bool isOutBound[4]);

  /**
   * Compute wheel speeds with compass correction and out-of-bounds prevention
   * `mAngle`          - Movement angle `(0-360°)`
   * `mSpeed`          - Movement speed `(0-255)`
   * `compassReading`  - Current compass heading `(0-360°)`
   * `isOutBound`      - Array indicating if each direction is out of bounds (Front, Right, Back, Left)
   *
   * `Returns` - Wheel speeds that combine compass correction and out-of-bounds prevention
   */
  WheelSpeeds correctedMove(float mAngle, float mSpeed, float compassReading, bool isOutBound[4]);

  /** Movement coordinate system, adjustable via converter */
  Converter converter;

  /** Compass correction PID controller */
  PIDController motorPID;
};

#endif // MOVEMENT_H
