#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#include "PeanutKingDef.h"

// #define  MOTOR     0xf0    // 4 motors * 2byte
// #define  ENCODER   0xf8    // 4 encoder* byte

/*  ================== Motor Mapping ==================
 *  Motor +ve speed -> motor rotate counter-clockwise (CCW)
 *  All speed +ve -> robot rotate clockwise (CW)
 *  Default Motor Mapping | MotorId index | MotorPos index
 *  Right Front -> M1     |      0        |      0
 *  Right Back  -> M2     |      1        |      1
 *  Left Back   -> M3     |      2        |      2
 *  Left Front  -> M4     |      3        |      3
 *  =================================================== */

// Motor port enumeration (M1-M4)
enum MotorId : uint8_t
{
  M1 = 0, // default pos: Right Front
  M2 = 1, // default pos: Right Back
  M3 = 2, // default pos: Left Back
  M4 = 3, // default pos: Left Front
  MotorMaxCount = 4 // Total number of motors
};

enum MotorPos : uint8_t
{
  RightFront = 0,
  RightBack  = 1,
  LeftBack   = 2,
  LeftFront  = 3
};

// Motor class to control 4 motors ( `M1` - `M4` ) with direction and speed control
class Motor
{
  friend class PeanutKingSoccerV4;   // Only PeanutKingSoccerV4 may construct this module

private:
  const uint8_t _in1Pin[4]; // motors' ch1 pins
  const uint8_t _in2Pin[4]; // motors' ch2 pins

  // Motor mapping array to determine which motor is connected to which port ( `M1` / `M2` / `M3` / `M4` )
  // Order: `Right Front`, `Right Back`, `Left Back`, `Left Front` (MotorPos order)
  // Default mapping: `M1`, `M2`, `M3`, `M4` (no mapping applied)
  MotorId _motorMap[4];

  // change motor rotation direction, `true` = flip, `false` = normal
  bool _motorflip[4];

  // Constructor (accessible only to the friend PeanutKingSoccerV4)
  Motor();

public:
  // Initialize motor pins as OUTPUT (called by PeanutKingSoccerV4)
  void init(void);

  // Check if a motor port is valid
  bool portValidCheck(MotorId mi);
  /**
   * Convert a physical position (RightFront/RightBack/LeftBack/LeftFront)
   * to the corresponding motor port (`M1~M4`) based on the current motor mapping.
   * `pos` - Physical position (RightFront/RightBack/LeftBack/LeftFront)
   *
   * `Returns` - Motor port (M1~M4) corresponding to the given position based on the current motor mapping.
   */
  MotorId getPortFromPos(MotorPos pos);
  /**
   * Assign which motor port ( `M1` - `M4` ) controls which wheel position
   * `RightFront` - Motor port for right front wheel (default: `M1`)
   * `RightBack`  - Motor port for right back wheel (default: `M2`)
   * `LeftBack`   - Motor port for left back wheel (default: `M3`)
   * `LeftFront`  - Motor port for left front wheel (default: `M4`)
   */
  void mapPort(MotorId RightFront, MotorId RightBack, MotorId LeftBack, MotorId LeftFront);

  /**
   * Flip single motor rotation direction
   * `mi`      - Motor ID ( `M1` - `M4` )
   * `flip`    - `true`=flip, `false`=normal (default: `true`)
   */
  void flipDirection(MotorId mi, bool flip = true);

  /**
   * Set single motor speed
   * `mi`     - Motor ID ( `M1` - `M4` ) directly driven port
   * `speed`  - Speed `(0~255)`, positive=`CCW,` negative=`CW`, `0`=`brake`
   */
  void setSpeed(MotorId mi, int16_t speed);
  /**
   * Stop a single motor (brake mode)
   * `mi` - Motor ID ( `M1` - `M4` )
   */
  void stop(MotorId mi);
};

#endif // MOTOR_H