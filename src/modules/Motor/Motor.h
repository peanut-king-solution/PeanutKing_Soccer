#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#include "PeanutKingDef.h"

// #define  MOTOR     0xf0    // 4 motors * 2byte
// #define  ENCODER   0xf8    // 4 encoder* byte

/*  ================== Motor Mapping ==================
 *  Motor +ve speed -> motor rotate counter-clockwise (CCW)
 *  All speed +ve -> robot rotate clockwise (CW)
 *  Default Motor Mapping | index in in1Pin/in2Pin arrays
 *  Left Front  -> M1     |              0
 *  Right Front -> M2     |              1
 *  Right Back  -> M3     |              2
 *  Left Back   -> M4     |              3
 *  =================================================== */

// Motor ID enumeration for identifying motors
typedef enum : uint8_t
{
  M1 = 0, // Left Front
  M2 = 1, // Right Front
  M3 = 2, // Right Back
  M4 = 3, // Left Back
} MOTOR_ID;

// Motor class to control 4 motors ( `M1` - `M4` ) with direction and speed control
class Motor
{
private:
  const uint8_t in1Pin[4]; // motors' ch1 pins
  const uint8_t in2Pin[4]; // motors' ch2 pins

  // Motor mapping array to determine which motor is connected to which port ( `M1` / `M2` / `M3` / `M4` )
  // Order: `Left Front`, `Right Front`, `Right Back`, `Left Back`
  // Default mapping: `M1`, `M2`, `M3`, `M4` (no mapping applied)
  MOTOR_ID motorMap[4];
  
  // change motor rotation direction, `true` = flip, `false` = normal
  bool motorflip[4];

public:
  // Constructor
  Motor();

  // Initialize motor pins as OUTPUT
  void init(void);

  /**
   * Assign which motor port ( `M1` - `M4` ) controls which wheel position
   * `LeftFront`  - Motor port
   * `RightFront` - Motor port
   * `RightBack`  - Motor port
   * `LeftBack`   - Motor port
   */
  void mapMotors(MOTOR_ID LeftFront, MOTOR_ID RightFront, MOTOR_ID RightBack, MOTOR_ID LeftBack);

  /**
   * Flip single motor rotation direction
   * `mi`      - Motor ID ( `M1` - `M4` )
   * `flip`    - `true`=flip, `false`=normal (default: `true`)
   */
  void flipMotor(MOTOR_ID mi, bool flip = true);

  /**
   * Flip all motors rotation direction
   * `m1, m2, m3, m4` - `true`=flip, `false`=normal
   */
  void flipMotors(bool m1, bool m2, bool m3, bool m4);

  /**
   * Set single motor speed
   * `mi`     - Motor ID ( `M1` - `M4` )
   * `speed`  - Speed `(0~255)`, positive=`CCW,` negative=`CW`, `0`=`brake`
   */
  void setSpeed(MOTOR_ID mi, int16_t speed);

  /**
   * Stop all motors (brake mode)
   */
  void stopAll(void);

  /**
   * Test motors sequentially ( `M1` -> `M2` -> `M3` -> `M4` )
   * `speed` - Test speed `(0~255)`
   */
  void testAll(int16_t speed);
};

#endif // MOTOR_H